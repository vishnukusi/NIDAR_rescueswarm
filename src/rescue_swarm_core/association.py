"""Survivor spatial association and deduplication module for RescueSwarm.

Provides deterministic multi-drone detection association, distinguishing repeated
observations of the same survivor from distinct nearby survivors, managing
uncertainty-gated matching, same-frame mutual exclusion, and task lifecycle states.
"""

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Any, Dict, List, Optional, Tuple, Union

from .localization import GeolocationEstimate, haversine_distance_m
from .models import GeoPoint, RescueTask, TaskStatus
from .perception import BoundingBox, Detection


class AssociationStatus(str, Enum):
    """Classification of an association decision for an incoming observation."""
    NEW_SURVIVOR = "new_survivor"          # High confidence new survivor
    MATCH = "match"                        # Confident match to an active survivor
    VERIFIED_MATCH = "verified_match"      # Sighting of an already verified (rescued) survivor
    FAILED_RETRY = "failed_retry"          # Re-sighting of a survivor whose previous task failed
    AMBIGUOUS = "ambiguous"                # Proximate but exceeds uncertainty or conflict bounds
    DISTINCT_NEARBY = "distinct_nearby"    # Spatially close but confirmed distinct (e.g. same frame)


@dataclass(frozen=True)
class AssociationDecision:
    """Detailed result of an association query."""
    status: AssociationStatus
    matched_task_id: Optional[str] = None
    distance_m: Optional[float] = None
    is_ambiguous: bool = False
    ambiguity_reason: Optional[str] = None
    explanation: str = ""
    matched_by_track: bool = False


class SurvivorAssociator:
    """Associates sensor detections with existing survivor records using spatial
    proximity, uncertainty gating, same-frame constraints, and lifecycle state.
    """

    def __init__(
        self,
        match_distance_threshold_m: float = 10.0,
        max_association_uncertainty_m: float = 20.0,
        min_distinct_separation_m: float = 3.0,
        handle_failed_as_new: bool = True,
        reopen_verified: bool = False,
        intra_frame_iou_threshold: float = 0.3,
        max_tracklet_gap_s: Optional[float] = 5.0,
    ) -> None:
        """Initialize the survivor associator.

        Args:
            match_distance_threshold_m: Maximum ground distance (meters) to associate
                an observation with an existing survivor.
            max_association_uncertainty_m: Maximum uncertainty radius (meters) allowed
                for confident association. Above this, matches are flagged as AMBIGUOUS.
            min_distinct_separation_m: Minimum physical distance assumed between distinct
                individuals (for spatial ambiguity checks and distance fallback).
            handle_failed_as_new: If True, re-observation of a FAILED task triggers a
                new rescue task attempt (FAILED_RETRY) rather than permanently abandoning.
            reopen_verified: If False (default), re-observing a VERIFIED survivor records
                the observation without re-dispatching a duplicate delivery drone.
            intra_frame_iou_threshold: Minimum bounding-box IoU required between two
                detections in the same camera frame to treat them as the same person
                (duplicate box / jitter). Below this, same-frame detections are treated
                as distinct individuals even if ground projections are proximate.
            max_tracklet_gap_s: Maximum temporal gap (in seconds) permitted for visual
                track continuity. Gaps beyond this expire the visual tracklet, falling back
                to spatial matching and re-enabling proximity ambiguity checks. Default 5.0s.
        """
        if match_distance_threshold_m <= 0.0:
            raise ValueError("match_distance_threshold_m must be strictly positive")
        if max_association_uncertainty_m <= 0.0:
            raise ValueError("max_association_uncertainty_m must be strictly positive")
        if min_distinct_separation_m < 0.0:
            raise ValueError("min_distinct_separation_m must be non-negative")
        if not (0.0 <= intra_frame_iou_threshold <= 1.0):
            raise ValueError("intra_frame_iou_threshold must be between 0.0 and 1.0")
        if max_tracklet_gap_s is not None and max_tracklet_gap_s <= 0.0:
            raise ValueError("max_tracklet_gap_s must be strictly positive or None")

        self.match_distance_threshold_m = match_distance_threshold_m
        self.max_association_uncertainty_m = max_association_uncertainty_m
        self.min_distinct_separation_m = min_distinct_separation_m
        self.handle_failed_as_new = handle_failed_as_new
        self.reopen_verified = reopen_verified
        self.intra_frame_iou_threshold = intra_frame_iou_threshold
        self.max_tracklet_gap_s = max_tracklet_gap_s

    def associate(
        self,
        detection: Detection,
        geo_estimate: GeolocationEstimate,
        detected_by_drone_id: str,
        survivor_records: Dict[str, Any],
        frame_id: Optional[str] = None,
        timestamp: float = 0.0,
        is_known_distinct: bool = False,
        known_distinct_from: Optional[List[str]] = None,
    ) -> AssociationDecision:
        """Associate an incoming detection with existing survivor records.

        Args:
            detection: Object detection from perception model.
            geo_estimate: 3D ray-plane ground geolocation estimate with uncertainty.
            detected_by_drone_id: Identifier of the observing drone.
            survivor_records: Dict mapping task_id to SurvivorRecord.
            frame_id: Optional identifier of the video frame/image capture.
            timestamp: Observation timestamp.
            is_known_distinct: If True, caller declares this target is known distinct from all existing.
            known_distinct_from: Optional list of task IDs this detection cannot belong to.

        Returns:
            AssociationDecision describing the matching outcome.
        """
        # 1. Defensive input validation
        if not isinstance(detection, Detection):
            raise ValueError("detection must be a valid Detection instance")
        if geo_estimate is None or not isinstance(geo_estimate, GeolocationEstimate):
            raise ValueError("geo_estimate must be a valid GeolocationEstimate instance")
        if geo_estimate.location is None:
            raise ValueError("geo_estimate.location cannot be None")
        if not math.isfinite(geo_estimate.location.latitude) or not math.isfinite(geo_estimate.location.longitude):
            raise ValueError("geo_estimate coordinates must be finite numbers")
        if not math.isfinite(geo_estimate.uncertainty_radius_m) or geo_estimate.uncertainty_radius_m < 0.0:
            raise ValueError("geo_estimate.uncertainty_radius_m must be a non-negative finite number")

        # Caller declares target is unconditionally distinct
        if is_known_distinct:
            return AssociationDecision(
                status=AssociationStatus.NEW_SURVIVOR,
                explanation="Caller explicitly marked detection as known distinct.",
            )

        target_loc = geo_estimate.location
        distinct_ids = set(known_distinct_from or [])

        # Resolve scoped track identifier for local visual tracklet continuity
        scoped_track = detection.scoped_track_id
        if scoped_track is None and detection.track_id is not None:
            sess = detection.tracker_session_id or "default_session"
            scoped_track = f"{detected_by_drone_id}:{sess}:{detection.track_id}"

        # 2. Evaluate candidates within match_distance_threshold_m
        candidates: List[Tuple[str, Any, float, bool]] = []

        for task_id, record in survivor_records.items():
            # Skip if explicitly marked as distinct from this task
            if task_id in distinct_ids:
                continue

            existing_task = record.task
            if existing_task.location is None:
                continue

            dist = haversine_distance_m(target_loc, existing_task.location)

            # Check if this candidate shares an active visual tracklet with this observing drone
            rec_tracks = getattr(record, "active_scoped_tracks", {})
            last_seen = getattr(record, "last_seen_timestamp", 0.0)
            time_gap = (timestamp - last_seen) if (timestamp >= last_seen) else 0.0
            is_track_fresh = (
                self.max_tracklet_gap_s is None
                or time_gap <= self.max_tracklet_gap_s
                or timestamp == 0.0
            )
            has_track_match = bool(
                scoped_track
                and rec_tracks.get(detected_by_drone_id) == scoped_track
                and is_track_fresh
            )

            # Same-frame constraint:
            # If this record was already observed/created in the EXACT SAME frame from the SAME drone:
            # - We check 2D bounding-box IoU (if record.last_bbox exists):
            #   * IoU >= intra_frame_iou_threshold: Duplicate box of the SAME person (e.g. NMS jitter).
            #     Must be considered a candidate match regardless of ground projection distance.
            #   * IoU < intra_frame_iou_threshold: Separate, non-overlapping detections in the same image.
            #     A single individual cannot be in two distinct non-overlapping boxes in a single frame.
            #     Therefore, they are distinct individuals and MUST NOT be merged, even if ground ray
            #     estimates are close (< 3.0m).
            # - Fallback (if last_bbox is not available):
            #   * If dist >= min_distinct_separation_m: distinct individuals, skip matching.
            if (
                frame_id is not None
                and getattr(record, "last_frame_id", None) == frame_id
                and getattr(record, "last_detected_by", None) == detected_by_drone_id
            ):
                rec_track = rec_tracks.get(detected_by_drone_id)
                if scoped_track and rec_track and scoped_track != rec_track:
                    # Two distinct active tracklets in the same image frame cannot be the same person
                    continue
                rec_bbox = getattr(record, "last_bbox", None)
                if rec_bbox is not None and isinstance(rec_bbox, BoundingBox):
                    iou = detection.bbox.iou(rec_bbox)
                    if iou < self.intra_frame_iou_threshold:
                        # Distinct person in the same frame
                        continue
                else:
                    if dist >= self.min_distinct_separation_m:
                        continue

            if dist <= self.match_distance_threshold_m:
                candidates.append((task_id, record, dist, has_track_match))

        # 3. No candidates within threshold -> Definitely a new survivor
        if not candidates:
            return AssociationDecision(
                status=AssociationStatus.NEW_SURVIVOR,
                explanation="No existing survivor task within spatial matching threshold.",
            )

        # 4. Sort candidates: matching local track continuity ranks first, then nearest distance
        candidates.sort(key=lambda c: (0 if c[3] else 1, c[2]))
        best_task_id, best_record, min_dist, best_has_track = candidates[0]

        # Helper to determine root task of a retry lineage
        def _get_lineage_root(t_id: str) -> str:
            curr = t_id
            seen = set()
            while curr in survivor_records and curr not in seen:
                seen.add(curr)
                parent = getattr(survivor_records[curr].task, "retry_of", None)
                if parent and parent in survivor_records:
                    curr = parent
                else:
                    break
            return curr

        best_root = _get_lineage_root(best_task_id)

        # 5. Check for ambiguity: multiple conflicting candidates from DIFFERENT lineages or excessive uncertainty
        is_ambiguous = False
        ambiguity_reasons = []

        other_lineage_candidates = [
            c for c in candidates if _get_lineage_root(c[0]) != best_root
        ]
        if other_lineage_candidates:
            second_task_id, second_rec, second_dist, second_has_track = other_lineage_candidates[0]
            if best_has_track:
                # Visual track continuity confirms identity on this camera stream.
                # Spatial proximity to another candidate does not cause ambiguity unless both claim same track ID.
                if second_has_track:
                    is_ambiguous = True
                    ambiguity_reasons.append(
                        f"Conflicting track continuity: {best_task_id} and {second_task_id} both claim track {scoped_track}"
                    )
            else:
                if abs(second_dist - min_dist) < self.min_distinct_separation_m:
                    is_ambiguous = True
                    ambiguity_reasons.append(
                        f"Conflicting candidates: near {best_task_id} ({min_dist:.1f}m) and {second_task_id} ({second_dist:.1f}m)"
                    )

        if geo_estimate.uncertainty_radius_m > self.max_association_uncertainty_m:
            is_ambiguous = True
            ambiguity_reasons.append(
                f"High observation uncertainty ({geo_estimate.uncertainty_radius_m:.1f}m > {self.max_association_uncertainty_m:.1f}m threshold)"
            )

        ambiguity_text = "; ".join(ambiguity_reasons) if ambiguity_reasons else None

        # 6. Resolve survivor lineage to prevent duplicate active retries
        # Find all records belonging to this survivor's lineage
        lineage_records = [
            r for r in survivor_records.values()
            if _get_lineage_root(r.task.task_id) == best_root
        ]

        # Check if there is an active retry task already underway for this survivor
        active_task_rec = next(
            (
                r for r in lineage_records
                if r.task.status in (
                    TaskStatus.GEOLOCATED,
                    TaskStatus.ASSIGNED,
                    TaskStatus.EN_ROUTE,
                    TaskStatus.RELEASED,
                )
            ),
            None,
        )

        # Check if there is a verified task for this survivor
        verified_task_rec = next(
            (r for r in lineage_records if r.task.status == TaskStatus.VERIFIED),
            None,
        )

        # Target record: prefer active task > verified task > latest candidate
        effective_rec = active_task_rec or verified_task_rec or best_record
        effective_task = effective_rec.task
        effective_task_id = effective_task.task_id

        # 7. Ambiguity check takes priority over confident lifecycle transitions
        if is_ambiguous:
            return AssociationDecision(
                status=AssociationStatus.AMBIGUOUS,
                matched_task_id=effective_task_id,
                distance_m=min_dist,
                is_ambiguous=True,
                ambiguity_reason=ambiguity_text,
                explanation="Observation is spatially proximate but ambiguous due to high uncertainty or multiple candidates.",
            )

        # 8. Apply Task Lifecycle rules

        # Case A: An active task is already underway for this survivor
        if effective_task.status in (
            TaskStatus.GEOLOCATED,
            TaskStatus.ASSIGNED,
            TaskStatus.EN_ROUTE,
            TaskStatus.RELEASED,
        ):
            return AssociationDecision(
                status=AssociationStatus.MATCH,
                matched_task_id=effective_task_id,
                distance_m=min_dist,
                is_ambiguous=False,
                ambiguity_reason=None,
                explanation=f"Confident spatial match to active task {effective_task_id} (distance={min_dist:.2f}m).",
                matched_by_track=best_has_track,
            )

        # Case B: Survivor was already VERIFIED (rescued)
        if effective_task.status == TaskStatus.VERIFIED:
            if self.reopen_verified:
                status = AssociationStatus.MATCH
                explanation = "Matched verified survivor; configured to reopen."
            else:
                status = AssociationStatus.VERIFIED_MATCH
                explanation = "Matched already verified survivor; recorded sighting without reopening task."
            return AssociationDecision(
                status=status,
                matched_task_id=effective_task_id,
                distance_m=min_dist,
                is_ambiguous=False,
                ambiguity_reason=None,
                explanation=explanation,
                matched_by_track=best_has_track,
            )

        # Case C: All tasks in lineage FAILED (delivery timed out or aborted)
        if effective_task.status == TaskStatus.FAILED:
            if self.handle_failed_as_new:
                return AssociationDecision(
                    status=AssociationStatus.FAILED_RETRY,
                    matched_task_id=effective_task_id,
                    distance_m=min_dist,
                    is_ambiguous=False,
                    ambiguity_reason=None,
                    explanation="Matched previously failed rescue task; triggering new rescue attempt.",
                    matched_by_track=best_has_track,
                )
            else:
                return AssociationDecision(
                    status=AssociationStatus.MATCH,
                    matched_task_id=effective_task_id,
                    distance_m=min_dist,
                    is_ambiguous=False,
                    ambiguity_reason=None,
                    explanation="Matched failed task; recorded without creating new task.",
                    matched_by_track=best_has_track,
                )

        # Case D: Fallback confident match
        return AssociationDecision(
            status=AssociationStatus.MATCH,
            matched_task_id=effective_task_id,
            distance_m=min_dist,
            is_ambiguous=False,
            ambiguity_reason=None,
            explanation=f"Confident spatial match to task {effective_task_id} (distance={min_dist:.2f}m).",
            matched_by_track=best_has_track,
        )
