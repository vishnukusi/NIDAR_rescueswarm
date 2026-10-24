"""Swarm manager coordinating search drones (S1-S3) and delivery drones (D1-D3).

Maintains survivor records, handles spatial deduplication of repeated detections,
manages task queues, and allocates delivery drones using the existing deterministic allocator.
"""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

from .allocator import AllocationError, assign_delivery_drone
from .association import AssociationDecision, AssociationStatus, SurvivorAssociator
from .localization import GeolocationEstimate, haversine_distance_m
from .mission import MissionManager
from .models import (
    Drone,
    DroneRole,
    DroneStatus,
    GeoPoint,
    MissionState,
    RescueTask,
    TaskStatus,
)
from .perception import BoundingBox, Detection


@dataclass
class SurvivorRecord:
    """Record of a verified or candidate survivor including detection history."""
    task: RescueTask
    uncertainty_radius_m: float
    detection_count: int = 1
    confirming_drones: List[str] = field(default_factory=list)
    last_frame_id: Optional[str] = None
    last_detected_by: Optional[str] = None
    last_seen_timestamp: float = 0.0
    ambiguous_sightings: int = 0
    ambiguity_notes: List[str] = field(default_factory=list)
    verified_re_sightings: int = 0
    last_bbox: Optional[BoundingBox] = None
    active_scoped_tracks: Dict[str, str] = field(default_factory=dict)


class SwarmManager:
    """Central coordination node connecting perception, tasks, and fleet allocation."""

    def __init__(
        self,
        drones: Optional[List[Drone]] = None,
        mission_manager: Optional[MissionManager] = None,
        duplicate_distance_threshold_m: float = 10.0,
        associator: Optional[SurvivorAssociator] = None,
    ) -> None:
        self.drones: Dict[str, Drone] = {}
        if drones:
            for drone in drones:
                self.register_drone(drone)

        self.mission_manager = mission_manager or MissionManager()
        self.duplicate_distance_threshold_m = duplicate_distance_threshold_m
        self.associator = associator or SurvivorAssociator(
            match_distance_threshold_m=duplicate_distance_threshold_m
        )
        self.survivor_records: Dict[str, SurvivorRecord] = {}
        self._next_task_index = 1

    @classmethod
    def from_config_file(cls, config_path: Union[str, Path]) -> "SwarmManager":
        """Initialize swarm manager and fleet from a fleet.json config file."""
        data = json.loads(Path(config_path).read_text())
        drones = []
        for item in data.get("fleet", []):
            role = DroneRole(item["role"])
            status = DroneStatus.READY if role == DroneRole.DELIVERY else DroneStatus.STANDBY
            drones.append(Drone(drone_id=item["id"], role=role, status=status))

        assoc_data = data.get("association", {})
        associator = None
        if assoc_data:
            associator = SurvivorAssociator(
                match_distance_threshold_m=assoc_data.get("match_distance_threshold_m", 10.0),
                max_association_uncertainty_m=assoc_data.get("max_association_uncertainty_m", 20.0),
                min_distinct_separation_m=assoc_data.get("min_distinct_separation_m", 3.0),
                intra_frame_iou_threshold=assoc_data.get("intra_frame_iou_threshold", 0.3),
                max_tracklet_gap_s=assoc_data.get("max_tracklet_gap_s", 5.0),
            )
        return cls(drones=drones, associator=associator)

    def register_drone(self, drone: Drone) -> None:
        """Register or update a drone in the swarm."""
        self.drones[drone.drone_id] = drone

    def get_drone(self, drone_id: str) -> Optional[Drone]:
        """Retrieve drone by ID."""
        return self.drones.get(drone_id)

    def _bind_scoped_track(
        self,
        target_task_id: str,
        drone_id: str,
        scoped_track: Optional[str],
    ) -> None:
        """Bind a scoped track to a survivor record and evict stale bindings from other records.

        Guarantees that a scoped visual tracklet from a specific drone camera belongs
        to at most one survivor record at any given time.
        """
        if not scoped_track:
            return
        if not scoped_track.startswith(f"{drone_id}:"):
            return

        for other_id, other_rec in self.survivor_records.items():
            if other_id != target_task_id and other_rec.active_scoped_tracks.get(drone_id) == scoped_track:
                other_rec.active_scoped_tracks.pop(drone_id, None)

        if target_task_id in self.survivor_records:
            self.survivor_records[target_task_id].active_scoped_tracks[drone_id] = scoped_track

    def process_detection(
        self,
        detection: Detection,
        geo_estimate: GeolocationEstimate,
        detected_by_drone_id: str,
        minimum_confidence: float = 0.50,
        frame_id: Optional[str] = None,
        timestamp: float = 0.0,
        is_known_distinct: bool = False,
        known_distinct_from: Optional[List[str]] = None,
    ) -> Tuple[RescueTask, bool]:
        """Process a validated perception detection and geolocation estimate.

        Uses SurvivorAssociator to distinguish repeated sightings from distinct
        nearby survivors, enforce same-frame constraints, gate high uncertainty,
        and handle task lifecycle states.

        Returns:
            Tuple[RescueTask, bool]: (task, is_new_task)
        """
        if detection is None or not isinstance(detection, Detection):
            raise ValueError("detection must be a valid Detection instance")
        if detection.confidence < minimum_confidence:
            raise ValueError(
                f"Detection confidence {detection.confidence:.2f} below threshold {minimum_confidence:.2f}"
            )

        decision = self.associator.associate(
            detection=detection,
            geo_estimate=geo_estimate,
            detected_by_drone_id=detected_by_drone_id,
            survivor_records=self.survivor_records,
            frame_id=frame_id,
            timestamp=timestamp,
            is_known_distinct=is_known_distinct,
            known_distinct_from=known_distinct_from,
        )

        # Resolve scoped visual tracklet ID for this observing drone
        scoped_track = detection.scoped_track_id
        if scoped_track is None and detection.track_id is not None:
            sess = detection.tracker_session_id or "default_session"
            scoped_track = f"{detected_by_drone_id}:{sess}:{detection.track_id}"

        # 1. Confident match to an active task
        if decision.status == AssociationStatus.MATCH:
            record = self.survivor_records[decision.matched_task_id]
            existing_task = record.task
            record.detection_count += 1
            if detected_by_drone_id not in record.confirming_drones:
                record.confirming_drones.append(detected_by_drone_id)

            if detection.confidence > existing_task.confidence:
                existing_task.confidence = detection.confidence

            if geo_estimate.uncertainty_radius_m < record.uncertainty_radius_m:
                existing_task.location = geo_estimate.location
                record.uncertainty_radius_m = geo_estimate.uncertainty_radius_m

            record.last_frame_id = frame_id
            record.last_detected_by = detected_by_drone_id
            record.last_seen_timestamp = timestamp
            record.last_bbox = detection.bbox
            self._bind_scoped_track(existing_task.task_id, detected_by_drone_id, scoped_track)
            return existing_task, False

        # 2. Sighting of an already verified survivor (prevent duplicate delivery)
        if decision.status == AssociationStatus.VERIFIED_MATCH:
            record = self.survivor_records[decision.matched_task_id]
            existing_task = record.task
            record.detection_count += 1
            record.verified_re_sightings += 1
            if detected_by_drone_id not in record.confirming_drones:
                record.confirming_drones.append(detected_by_drone_id)

            if detection.confidence > existing_task.confidence:
                existing_task.confidence = detection.confidence

            record.last_frame_id = frame_id
            record.last_detected_by = detected_by_drone_id
            record.last_seen_timestamp = timestamp
            record.last_bbox = detection.bbox
            self._bind_scoped_track(existing_task.task_id, detected_by_drone_id, scoped_track)
            return existing_task, False

        # 3. Ambiguous sighting (high uncertainty or multiple competing candidates)
        if decision.status == AssociationStatus.AMBIGUOUS:
            record = self.survivor_records[decision.matched_task_id]
            existing_task = record.task
            record.detection_count += 1
            record.ambiguous_sightings += 1
            if decision.ambiguity_reason:
                record.ambiguity_notes.append(decision.ambiguity_reason)
            if detected_by_drone_id not in record.confirming_drones:
                record.confirming_drones.append(detected_by_drone_id)

            # Do not overwrite refined location with coarse/ambiguous estimate
            record.last_frame_id = frame_id
            record.last_detected_by = detected_by_drone_id
            record.last_seen_timestamp = timestamp
            record.last_bbox = detection.bbox
            self._bind_scoped_track(existing_task.task_id, detected_by_drone_id, scoped_track)
            return existing_task, False

        # 4. Re-sighting of a failed task (retry delivery rather than abandon)
        if decision.status == AssociationStatus.FAILED_RETRY:
            task_id = f"task_{self._next_task_index:03d}"
            self._next_task_index += 1

            new_task = RescueTask(
                task_id=task_id,
                confidence=detection.confidence,
                image_x=detection.bbox.center_x,
                image_y=detection.bbox.center_y,
                detected_by=detected_by_drone_id,
                status=TaskStatus.GEOLOCATED,
                location=geo_estimate.location,
                retry_of=decision.matched_task_id,
            )
            record = SurvivorRecord(
                task=new_task,
                uncertainty_radius_m=geo_estimate.uncertainty_radius_m,
                detection_count=1,
                confirming_drones=[detected_by_drone_id],
                last_frame_id=frame_id,
                last_detected_by=detected_by_drone_id,
                last_seen_timestamp=timestamp,
                last_bbox=detection.bbox,
                active_scoped_tracks={},
            )
            self.survivor_records[task_id] = record
            self._bind_scoped_track(task_id, detected_by_drone_id, scoped_track)
            self.dispatch_pending_tasks()
            if (
                self.mission_manager.state == MissionState.SEARCHING
                and any(t.status == TaskStatus.ASSIGNED for t in self.get_all_tasks())
            ):
                self.mission_manager.transition_to(MissionState.RESCUE_IN_PROGRESS)
            return new_task, True

        # 5. New survivor detected
        task_id = f"task_{self._next_task_index:03d}"
        self._next_task_index += 1

        new_task = RescueTask(
            task_id=task_id,
            confidence=detection.confidence,
            image_x=detection.bbox.center_x,
            image_y=detection.bbox.center_y,
            detected_by=detected_by_drone_id,
            status=TaskStatus.GEOLOCATED,
            location=geo_estimate.location,
        )
        record = SurvivorRecord(
            task=new_task,
            uncertainty_radius_m=geo_estimate.uncertainty_radius_m,
            detection_count=1,
            confirming_drones=[detected_by_drone_id],
            last_frame_id=frame_id,
            last_detected_by=detected_by_drone_id,
            last_seen_timestamp=timestamp,
            last_bbox=detection.bbox,
            active_scoped_tracks={},
        )
        self.survivor_records[task_id] = record
        self._bind_scoped_track(task_id, detected_by_drone_id, scoped_track)

        # Attempt immediate delivery drone assignment
        self.dispatch_pending_tasks()

        # Update mission state if not already in rescue phase
        if (
            self.mission_manager.state == MissionState.SEARCHING
            and any(t.status == TaskStatus.ASSIGNED for t in self.get_all_tasks())
        ):
            self.mission_manager.transition_to(MissionState.RESCUE_IN_PROGRESS)

        return new_task, True

    def process_detections_batch(
        self,
        detections_with_geo: List[Tuple[Detection, GeolocationEstimate]],
        detected_by_drone_id: str,
        frame_id: Optional[str] = None,
        timestamp: float = 0.0,
        minimum_confidence: float = 0.50,
    ) -> List[Tuple[RescueTask, bool]]:
        """Atomically process multiple detections from the same capture event (frame).

        Enforces transactional atomicity with validation-before-mutation and rollback:
        1. Pre-validates every detection, bounding box, geolocation estimate, and
           parameter across the entire batch BEFORE modifying any internal state.
        2. If any element in the batch fails validation, a ValueError is raised and
           NO tasks are created, NO drones are assigned, and NO mission transitions occur.
        3. If an unexpected error occurs during execution, the manager state is restored
           to the exact snapshot taken prior to batch execution.
        """
        if not isinstance(detections_with_geo, list):
            raise ValueError(f"detections_with_geo must be a list, got {type(detections_with_geo)}")

        if not detections_with_geo:
            return []

        if not detected_by_drone_id:
            raise ValueError("detected_by_drone_id cannot be empty")

        # 1. Strict pre-validation of all items prior to state mutation
        for idx, item in enumerate(detections_with_geo):
            if not isinstance(item, (tuple, list)) or len(item) != 2:
                raise ValueError(f"Batch item #{idx} must be a (Detection, GeolocationEstimate) tuple")

            det, geo = item
            if not isinstance(det, Detection):
                raise ValueError(f"Batch item #{idx} contains an invalid Detection: {type(det)}")
            if det.confidence < minimum_confidence:
                raise ValueError(
                    f"Batch item #{idx} detection confidence {det.confidence:.2f} below threshold {minimum_confidence:.2f}"
                )
            if not isinstance(geo, GeolocationEstimate):
                raise ValueError(f"Batch item #{idx} contains an invalid GeolocationEstimate: {type(geo)}")
            if geo.location is None:
                raise ValueError(f"Batch item #{idx} geo_estimate.location cannot be None")
            if not (math.isfinite(geo.location.latitude) and math.isfinite(geo.location.longitude)):
                raise ValueError(f"Batch item #{idx} geo_estimate contains non-finite coordinates")
            if not math.isfinite(geo.uncertainty_radius_m) or geo.uncertainty_radius_m < 0.0:
                raise ValueError(f"Batch item #{idx} geo_estimate uncertainty must be non-negative finite")

        # 2. Capture transactional snapshot for rollback guarantee
        snapshot_records = {
            t_id: SurvivorRecord(
                task=RescueTask(
                    task_id=r.task.task_id,
                    confidence=r.task.confidence,
                    image_x=r.task.image_x,
                    image_y=r.task.image_y,
                    detected_by=r.task.detected_by,
                    status=r.task.status,
                    location=r.task.location,
                    assigned_drone_id=r.task.assigned_drone_id,
                    retry_of=r.task.retry_of,
                ),
                uncertainty_radius_m=r.uncertainty_radius_m,
                detection_count=r.detection_count,
                confirming_drones=list(r.confirming_drones),
                last_frame_id=r.last_frame_id,
                last_detected_by=r.last_detected_by,
                last_seen_timestamp=r.last_seen_timestamp,
                ambiguous_sightings=r.ambiguous_sightings,
                ambiguity_notes=list(r.ambiguity_notes),
                verified_re_sightings=r.verified_re_sightings,
                last_bbox=r.last_bbox,
                active_scoped_tracks=dict(r.active_scoped_tracks),
            )
            for t_id, r in self.survivor_records.items()
        }
        snapshot_task_idx = self._next_task_index
        snapshot_drones = {
            d_id: (d.status, d.active_task_id)
            for d_id, d in self.drones.items()
        }
        snapshot_mission_state = self.mission_manager.state

        results: List[Tuple[RescueTask, bool]] = []
        try:
            for det, geo in detections_with_geo:
                task, is_new = self.process_detection(
                    detection=det,
                    geo_estimate=geo,
                    detected_by_drone_id=detected_by_drone_id,
                    minimum_confidence=minimum_confidence,
                    frame_id=frame_id,
                    timestamp=timestamp,
                )
                results.append((task, is_new))
            return results
        except Exception:
            # Transaction rollback: restore exact pre-batch state
            self.survivor_records = snapshot_records
            self._next_task_index = snapshot_task_idx
            for d_id, (st, at) in snapshot_drones.items():
                if d_id in self.drones:
                    self.drones[d_id].status = st
                    self.drones[d_id].active_task_id = at
            self.mission_manager.state = snapshot_mission_state
            raise

    def dispatch_pending_tasks(self) -> List[Tuple[str, str]]:
        """Try allocating available delivery drones to unassigned geolocated tasks.

        Returns list of (task_id, assigned_drone_id).
        """
        assignments: List[Tuple[str, str]] = []
        unassigned_tasks = [
            r.task for r in self.survivor_records.values()
            if r.task.status == TaskStatus.GEOLOCATED and r.task.assigned_drone_id is None
        ]

        for task in unassigned_tasks:
            try:
                assigned_drone_id = assign_delivery_drone(task, self.drones.values())
                assignments.append((task.task_id, assigned_drone_id))
            except AllocationError:
                # No eligible drone currently available, task remains in GEOLOCATED queue
                break

        return assignments

    def update_task_status(
        self,
        task_id: str,
        new_status: TaskStatus,
        drone_available: Optional[bool] = None,
        drone_status: Optional[DroneStatus] = None,
    ) -> RescueTask:
        """Advance the task status lifecycle and handle drone status transitions.

        Maintains strict separation between task lifecycle and vehicle safety state:
        - When drone_status is provided, the drone is set to that explicit status.
        - When drone_available is False, the vehicle is marked RETURNING (recovering)
          and will NOT be allocated to waiting tasks.
        - When drone_available is True, the vehicle is confirmed available (READY).
        - If drone is already in FAULT or OFFLINE, marking task FAILED or VERIFIED
          will NEVER prematurely clear the fault to READY.
        - By default (drone_available is None), releases the task and resets healthy
          vehicles to READY for backward compatibility with existing callers.
        """
        if task_id not in self.survivor_records:
            raise KeyError(f"Task {task_id} not found")

        task = self.survivor_records[task_id].task
        task.status = new_status

        # If task reached terminal status (VERIFIED or FAILED), disassociate task from drone
        if new_status in (TaskStatus.VERIFIED, TaskStatus.FAILED):
            if task.assigned_drone_id:
                drone = self.drones.get(task.assigned_drone_id)
                if drone:
                    drone.active_task_id = None
                    if drone_status is not None:
                        # Safety rule: A generic READY transition must never overwrite a safety-critical state (FAULT, OFFLINE)
                        if drone.status in (DroneStatus.FAULT, DroneStatus.OFFLINE) and drone_status == DroneStatus.READY:
                            pass
                        elif drone_status == DroneStatus.READY and drone.battery_fraction <= 0.20:
                            # A depleted or unsafe battery drone must not be set to READY
                            drone.status = DroneStatus.STANDBY
                        else:
                            drone.status = drone_status
                    elif drone_available is False:
                        if drone.status not in (DroneStatus.FAULT, DroneStatus.OFFLINE):
                            drone.status = DroneStatus.RETURNING
                    elif drone_available is True:
                        if drone.status not in (DroneStatus.FAULT, DroneStatus.OFFLINE):
                            if drone.battery_fraction > 0.20:
                                drone.status = DroneStatus.READY
                            else:
                                drone.status = DroneStatus.STANDBY
                    else:
                        # drone_available is None: default release unless in fault/offline/depleted battery
                        if drone.status not in (DroneStatus.FAULT, DroneStatus.OFFLINE):
                            if drone.battery_fraction > 0.20:
                                drone.status = DroneStatus.READY
                            else:
                                drone.status = DroneStatus.STANDBY

            # Attempt to dispatch any waiting tasks only if at least one delivery drone is READY
            if any(d.role == DroneRole.DELIVERY and d.status == DroneStatus.READY for d in self.drones.values()):
                self.dispatch_pending_tasks()

        return task

    def recover_drone(self, drone_id: str, new_status: DroneStatus = DroneStatus.READY) -> bool:
        """Explicitly transition a drone out of a safety-critical state (FAULT, OFFLINE).

        Authorization note: This method provides a dedicated programmatic recovery hook.
        It does NOT enforce cryptographic authorization, tokens, or role-based access
        control (RBAC) at the software level. Any runtime caller with access to the
        SwarmManager instance may invoke this method to reset drone status.
        """
        drone = self.drones.get(drone_id)
        if not drone:
            return False
        # Do not allow recovery to READY if battery is critically depleted
        if new_status == DroneStatus.READY and drone.battery_fraction <= 0.20:
            drone.status = DroneStatus.STANDBY
            return False
        drone.status = new_status
        return True

    def get_all_tasks(self) -> List[RescueTask]:
        """Return all tracked rescue tasks."""
        return [r.task for r in self.survivor_records.values()]

    def get_active_tasks(self) -> List[RescueTask]:
        """Return tasks currently in progress."""
        return [
            r.task for r in self.survivor_records.values()
            if r.task.status in (TaskStatus.GEOLOCATED, TaskStatus.ASSIGNED, TaskStatus.EN_ROUTE, TaskStatus.RELEASED)
        ]

    def get_fleet_summary(self) -> dict:
        """Return high-level status summary of the swarm and missions."""
        tasks = self.get_all_tasks()
        return {
            "mission_state": self.mission_manager.state.value,
            "total_survivors_found": len(tasks),
            "verified_deliveries": sum(1 for t in tasks if t.status == TaskStatus.VERIFIED),
            "pending_tasks": sum(1 for t in tasks if t.status == TaskStatus.GEOLOCATED),
            "active_tasks": sum(1 for t in tasks if t.status in (TaskStatus.ASSIGNED, TaskStatus.EN_ROUTE, TaskStatus.RELEASED)),
            "drones": {
                d_id: {
                    "role": d.role.value,
                    "status": d.status.value,
                    "battery": d.battery_fraction,
                    "active_task": d.active_task_id,
                }
                for d_id, d in self.drones.items()
            },
        }
