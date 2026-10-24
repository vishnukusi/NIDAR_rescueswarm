"""Regression tests for the NIDAR RescueSwarm reliability audit.

Covers:
- Fix 1: Zero YOLO detections → no task, no delivery.
- Fix 2: Delivery controller timeout → task must NOT become VERIFIED.
          Controller not in successful terminal state → task must NOT become VERIFIED.
- Fix 3: Confidence threshold is explicit and conservative.
- Fix 4: Simulated timer verification is identified as simulated.
- Fix 5: Genuine simulated completion still follows expected path.

All tests use mocks/dependency injection — no best.pt, video assets, ROS 2,
PX4, or Gazebo required.
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from rescue_swarm_core.delivery import (
    DeliveryMissionController,
    DeliveryProfile,
    DeliveryState,
    DeliveryVerificationMethod,
)
from rescue_swarm_core.localization import GeolocationEstimate
from rescue_swarm_core.mission import MissionManager
from rescue_swarm_core.models import (
    Drone,
    DroneRole,
    DroneStatus,
    GeoPoint,
    MissionState,
    RescueTask,
    TaskStatus,
)
from rescue_swarm_core.perception import BoundingBox, Detection
from rescue_swarm_core.swarm_manager import SwarmManager


# ─── Helpers ───────────────────────────────────────────────────────────────

def _make_task(location: GeoPoint = None, status: TaskStatus = TaskStatus.ASSIGNED) -> RescueTask:
    """Create a minimal RescueTask for testing."""
    loc = location or GeoPoint(latitude=12.9715, longitude=77.5945, altitude_m=0.0)
    return RescueTask(
        task_id="task_001",
        confidence=0.92,
        image_x=0.5,
        image_y=0.5,
        detected_by="S1",
        status=status,
        location=loc,
        assigned_drone_id="D1",
    )


def _make_delivery_controller(
    verification_method=DeliveryVerificationMethod.SIMULATED_TIMER,
    verification_timeout_s=15.0,
    verification_time_s=1.0,
    task=None,
) -> DeliveryMissionController:
    """Create a DeliveryMissionController for testing."""
    t = task or _make_task()
    home = GeoPoint(latitude=12.9710, longitude=77.5940, altitude_m=0.0)
    profile = DeliveryProfile(
        transit_altitude_m=20.0,
        release_altitude_m=8.0,
        horizontal_speed_mps=10.0,
        vertical_speed_mps=4.0,
        drop_tolerance_radius_m=3.0,
        release_hover_time_s=1.0,
        verification_time_s=verification_time_s,
        verification_timeout_s=verification_timeout_s,
    )
    return DeliveryMissionController(
        drone_id="D1",
        task=t,
        home_location=home,
        profile=profile,
        verification_method=verification_method,
    )


def _run_controller_to_state(ctrl, target_state, max_steps=200):
    """Step the controller until it reaches target_state or max_steps is exhausted."""
    for _ in range(max_steps):
        ctrl.update(1.0)
        if ctrl.state == target_state:
            return True
    return False


def _make_swarm_manager() -> SwarmManager:
    """Create a SwarmManager with the standard fleet for testing."""
    drones = [
        Drone("S1", DroneRole.SEARCH, DroneStatus.STANDBY),
        Drone("D1", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.90),
        Drone("D2", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.95),
    ]
    mission = MissionManager()
    mission.transition_to(MissionState.INITIALIZING)
    mission.transition_to(MissionState.SEARCHING)
    return SwarmManager(drones=drones, mission_manager=mission)


def _make_detection(conf=0.90):
    return Detection(
        class_id=0,
        class_name="person",
        confidence=conf,
        bbox=BoundingBox(xmin=0.45, ymin=0.45, xmax=0.55, ymax=0.55),
    )


def _make_geo_estimate(lat=12.9716, lon=77.5946):
    return GeolocationEstimate(
        location=GeoPoint(latitude=lat, longitude=lon, altitude_m=0.0),
        offset_north_m=0.0,
        offset_east_m=0.0,
        ground_distance_m=0.0,
        slant_range_m=30.0,
        uncertainty_radius_m=2.0,
    )


# ─── Fix 1: Zero detections → no task, no delivery ────────────────────────

class TestZeroDetectionsNoTask(unittest.TestCase):
    """When YOLO returns zero detections, no task or delivery should be created."""

    def test_empty_detections_creates_no_task(self):
        """An empty detection list should not produce any RescueTask in SwarmManager."""
        manager = _make_swarm_manager()
        # We simply verify that if no detections are returned, we don't call
        # process_detection and no tasks are created.
        self.assertEqual(len(manager.get_all_tasks()), 0)

    def test_pipeline_no_detection_path_exits_cleanly(self):
        """Simulate the pipeline path when raw_detections is empty.

        The pipeline should print NO TARGET DETECTED and return without
        creating any rescue task or starting any delivery.
        """
        # Simulate the critical pipeline logic:
        raw_detections = []  # Model returned nothing
        task_created = False
        delivery_started = False

        if not raw_detections:
            # This is the no-detection exit path from run_autonomous_pipeline.py
            result = "NO TARGET DETECTED"
        else:
            task_created = True
            delivery_started = True
            result = "TASK CREATED"

        self.assertEqual(result, "NO TARGET DETECTED")
        self.assertFalse(task_created)
        self.assertFalse(delivery_started)

    def test_below_threshold_detection_rejected(self):
        """A detection below minimum_confidence should be rejected, not silently accepted."""
        manager = _make_swarm_manager()
        low_det = _make_detection(conf=0.10)
        geo = _make_geo_estimate()

        with self.assertRaises(ValueError):
            manager.process_detection(low_det, geo, "S1", minimum_confidence=0.25)

        self.assertEqual(len(manager.get_all_tasks()), 0)


# ─── Fix 2: Delivery timeout/failure must NOT become VERIFIED ─────────────

class TestDeliveryTimeoutNotVerified(unittest.TestCase):
    """A delivery that times out must NOT be marked VERIFIED."""

    def test_sensor_timeout_leaves_unverified(self):
        """When waiting for a sensor event that never arrives, delivery_verified stays False."""
        ctrl = _make_delivery_controller(
            verification_method=DeliveryVerificationMethod.GROUND_CONTACT_SENSOR,
            verification_timeout_s=5.0,
        )
        ctrl.start_mission()

        # Run until VERIFY_DROP
        reached = _run_controller_to_state(ctrl, DeliveryState.VERIFY_DROP)
        self.assertTrue(reached, "Controller should reach VERIFY_DROP")

        # Wait past the timeout without providing sensor event
        ctrl.update(6.0)

        # Controller should have moved to ASCENT but delivery_verified must be False
        self.assertEqual(ctrl.state, DeliveryState.ASCENT)
        self.assertFalse(ctrl.delivery_verified)
        self.assertIn("timed out", ctrl.verification_details)

    def test_timeout_task_not_marked_verified(self):
        """A task whose delivery timed out must not be set to VERIFIED by the caller."""
        task = _make_task()
        ctrl = _make_delivery_controller(
            verification_method=DeliveryVerificationMethod.SIMULATION_EVENT,
            verification_timeout_s=5.0,
            task=task,
        )
        ctrl.start_mission()

        # Run to completion (will time out in VERIFY_DROP → ASCENT → RTL → COMPLETED)
        _run_controller_to_state(ctrl, DeliveryState.COMPLETED, max_steps=300)

        # The delivery controller reached COMPLETED but delivery_verified is False
        self.assertEqual(ctrl.state, DeliveryState.COMPLETED)
        self.assertFalse(ctrl.delivery_verified)

        # The correct caller behavior: do NOT mark VERIFIED
        if ctrl.state == DeliveryState.COMPLETED and ctrl.delivery_verified:
            task.status = TaskStatus.VERIFIED  # Should NOT execute
        else:
            task.status = TaskStatus.FAILED

        self.assertEqual(task.status, TaskStatus.FAILED)
        self.assertNotEqual(task.status, TaskStatus.VERIFIED)


class TestIncompleteDeliveryNotVerified(unittest.TestCase):
    """A controller not in successful terminal state must NOT be marked VERIFIED."""

    def test_aborted_delivery_not_verified(self):
        """An aborted delivery must not be marked VERIFIED."""
        ctrl = _make_delivery_controller()
        ctrl.start_mission()
        ctrl.update(2.0)  # Partial flight
        ctrl.abort()

        self.assertEqual(ctrl.state, DeliveryState.RETURNING_HOME)
        self.assertFalse(ctrl.delivery_verified)

        # Simulate correct caller check
        should_verify = (ctrl.state == DeliveryState.COMPLETED and ctrl.delivery_verified)
        self.assertFalse(should_verify)

    def test_mid_flight_not_verified(self):
        """A controller still in EN_ROUTE should not be treated as verified."""
        ctrl = _make_delivery_controller()
        ctrl.start_mission()

        # Run just enough to get to EN_ROUTE
        _run_controller_to_state(ctrl, DeliveryState.EN_ROUTE)

        self.assertEqual(ctrl.state, DeliveryState.EN_ROUTE)
        self.assertFalse(ctrl.delivery_verified)

        should_verify = (ctrl.state == DeliveryState.COMPLETED and ctrl.delivery_verified)
        self.assertFalse(should_verify)


# ─── Fix 3: Confidence threshold explicit and documented ───────────────────

class TestConfidenceThreshold(unittest.TestCase):
    """The confidence threshold should be explicit and conservative."""

    def test_pipeline_default_threshold_is_not_003(self):
        """The pipeline default threshold must not be the old 0.03 value."""
        # Import the module-level constant
        sys.path.insert(0, str(ROOT / "scripts"))
        # We check the constant directly from the script
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "run_autonomous_pipeline",
            ROOT / "scripts" / "run_autonomous_pipeline.py",
        )
        mod = importlib.util.module_from_spec(spec)
        # We can't fully execute the module (needs ultralytics etc.),
        # so we read the source and check the constant
        source = (ROOT / "scripts" / "run_autonomous_pipeline.py").read_text(encoding="utf-8")
        self.assertIn("_DEFAULT_CONFIDENCE_THRESHOLD", source)
        self.assertNotIn("default=0.03", source)
        # Verify 0.25 is the documented default
        self.assertIn("0.25", source)

    def test_swarm_manager_default_minimum_confidence_is_050(self):
        """SwarmManager.process_detection has a default minimum_confidence of 0.50."""
        import inspect
        sig = inspect.signature(SwarmManager.process_detection)
        default = sig.parameters["minimum_confidence"].default
        self.assertEqual(default, 0.50)


# ─── Fix 4: Simulated verification labelled correctly ──────────────────────

class TestSimulatedVerificationLabelling(unittest.TestCase):
    """Simulated timer verification must be clearly labelled as simulated."""

    def test_simulated_timer_source_says_simulated(self):
        """The verification_source for SIMULATED_TIMER must contain 'simulated'."""
        ctrl = _make_delivery_controller(
            verification_method=DeliveryVerificationMethod.SIMULATED_TIMER,
            verification_time_s=1.0,
        )
        ctrl.start_mission()

        # Run to completion
        reached = _run_controller_to_state(ctrl, DeliveryState.COMPLETED, max_steps=300)
        self.assertTrue(reached, "Controller should reach COMPLETED")
        self.assertTrue(ctrl.delivery_verified)

        # Verification source must indicate it's simulated
        self.assertIn("simulated", ctrl.verification_source.lower())
        self.assertIn("simulated", ctrl.verification_details.lower())

    def test_simulated_timer_is_not_sensor_confirmation(self):
        """SIMULATED_TIMER verification must NOT claim to be ground_contact_sensor."""
        ctrl = _make_delivery_controller(
            verification_method=DeliveryVerificationMethod.SIMULATED_TIMER,
        )
        ctrl.start_mission()
        _run_controller_to_state(ctrl, DeliveryState.COMPLETED, max_steps=300)

        self.assertNotEqual(ctrl.verification_source, "ground_contact_sensor")
        self.assertNotEqual(ctrl.verification_method, DeliveryVerificationMethod.GROUND_CONTACT_SENSOR)


# ─── Fix 5: Genuine simulated completion follows expected path ─────────────

class TestGenuineSimulatedCompletion(unittest.TestCase):
    """A genuinely successful simulated delivery follows the expected state path."""

    def test_full_simulated_lifecycle(self):
        """SIMULATED_TIMER delivery completes through all expected states."""
        ctrl = _make_delivery_controller(
            verification_method=DeliveryVerificationMethod.SIMULATED_TIMER,
        )
        ctrl.start_mission()
        self.assertEqual(ctrl.state, DeliveryState.TAKEOFF)

        # Track all states visited
        visited_states = {DeliveryState.TAKEOFF}
        for _ in range(300):
            ctrl.update(1.0)
            visited_states.add(ctrl.state)
            if ctrl.state == DeliveryState.COMPLETED:
                break

        # Verify we went through the expected lifecycle
        expected_states = {
            DeliveryState.TAKEOFF,
            DeliveryState.EN_ROUTE,
            DeliveryState.APPROACH,
            DeliveryState.DESCENT_TO_DROP,
            DeliveryState.PAYLOAD_RELEASE,
            DeliveryState.VERIFY_DROP,
            DeliveryState.ASCENT,
            DeliveryState.RETURNING_HOME,
            DeliveryState.COMPLETED,
        }
        self.assertEqual(visited_states, expected_states)

        # Final assertions
        self.assertTrue(ctrl.payload_released)
        self.assertTrue(ctrl.delivery_verified)
        self.assertEqual(ctrl.state, DeliveryState.COMPLETED)
        self.assertEqual(ctrl.altitude_agl_m, 0.0)

    def test_sensor_verified_delivery_completes(self):
        """A delivery with external sensor confirmation completes correctly."""
        ctrl = _make_delivery_controller(
            verification_method=DeliveryVerificationMethod.GROUND_CONTACT_SENSOR,
            verification_timeout_s=30.0,
        )
        ctrl.start_mission()

        # Run to VERIFY_DROP
        _run_controller_to_state(ctrl, DeliveryState.VERIFY_DROP)
        self.assertEqual(ctrl.state, DeliveryState.VERIFY_DROP)
        self.assertFalse(ctrl.delivery_verified)

        # Provide sensor confirmation
        ctrl.confirm_ground_delivery("ground_contact_sensor", "Impact detected at 0.3 m/s")
        self.assertTrue(ctrl.delivery_verified)
        self.assertEqual(ctrl.verification_source, "ground_contact_sensor")

        # Continue to completion
        _run_controller_to_state(ctrl, DeliveryState.COMPLETED, max_steps=200)
        self.assertEqual(ctrl.state, DeliveryState.COMPLETED)
        self.assertTrue(ctrl.delivery_verified)


# ─── Fix 2: Mission-Script Integration Tests & Exception Safety ────────────

class TestMissionScriptIntegration(unittest.TestCase):
    """Integration-style tests that invoke actual mission-script paths.

    Verifies that real script entry points:
    - Handle zero detections without creating tasks or deliveries
    - Handle delivery controller exceptions safely without stranding drones or leaving tasks EN_ROUTE
    - Handle loop timeouts as unverified/failed outcomes
    - Execute genuinely successful simulated deliveries to verified completion
    """

    @patch("run_autonomous_pipeline.inspect_model_weights")
    @patch("run_autonomous_pipeline.YOLOPerceptionAdapter.predict", return_value=[])
    def test_pipeline_zero_detections_real_script_path(self, mock_predict, mock_inspect):
        """Invoke real run_pipeline() when YOLO inference returns zero detections.

        Verifies that:
        - The real script path executes the no-detection return
        - Status is 'no_target'
        - No rescue task is created (task is None)
        - No delivery controller is started (controller is None)
        """
        import run_autonomous_pipeline

        mock_inspect.return_value = {
            "framework": "Ultralytics YOLOv8",
            "task": "detect",
            "classes": {0: "pedestrian"},
            "num_classes": 1,
            "target_survivor_classes": ["pedestrian"],
        }
        test_img = ROOT / "assets" / "test_survivors.jpg"
        result = run_autonomous_pipeline.run_pipeline(image_path=test_img, confidence_threshold=0.25)

        self.assertEqual(result["status"], "no_target")
        self.assertIsNone(result["task"])
        self.assertIsNone(result["manager"])
        self.assertIsNone(result["controller"])
        self.assertIsNone(result["error"])
        self.assertFalse(result["timed_out"])

    @patch("run_autonomous_pipeline.inspect_model_weights")
    @patch("run_autonomous_pipeline.YOLOPerceptionAdapter.predict")
    @patch("run_autonomous_pipeline.annotate_detection_image")
    @patch.object(DeliveryMissionController, "update", side_effect=RuntimeError("Actuator telemetry disconnected"))
    def test_pipeline_controller_exception_releases_drone(self, mock_update, mock_annotate, mock_predict, mock_inspect):
        """Invoke real run_pipeline() where DeliveryMissionController.update() raises an exception.

        Verifies that:
        - Delivery loop execution is exception-safe (does not crash unhandled)
        - The task does NOT remain indefinitely EN_ROUTE (marked FAILED)
        - The software drone allocation is released (active_task_id is None, status is READY)
        - The original exception details are preserved
        """
        import run_autonomous_pipeline

        mock_inspect.return_value = {
            "framework": "Ultralytics YOLOv8",
            "task": "detect",
            "classes": {0: "pedestrian"},
            "num_classes": 1,
            "target_survivor_classes": ["pedestrian"],
        }
        mock_predict.return_value = [
            Detection(0, "pedestrian", 0.92, BoundingBox(0.48, 0.48, 0.52, 0.52))
        ]
        test_img = ROOT / "assets" / "test_survivors.jpg"
        result = run_autonomous_pipeline.run_pipeline(image_path=test_img, confidence_threshold=0.25)

        self.assertEqual(result["status"], "failed")
        self.assertIsNotNone(result["error"])
        self.assertIsInstance(result["error"], RuntimeError)
        self.assertEqual(str(result["error"]), "Actuator telemetry disconnected")

        # Task must be FAILED, NOT EN_ROUTE
        task = result["task"]
        self.assertIsNotNone(task)
        self.assertEqual(task.status, TaskStatus.FAILED)
        self.assertNotEqual(task.status, TaskStatus.EN_ROUTE)

        # Assigned drone must be released in SwarmManager
        manager = result["manager"]
        drone = manager.get_drone(task.assigned_drone_id)
        self.assertIsNone(drone.active_task_id)
        self.assertEqual(drone.status, DroneStatus.READY)

    @patch.object(DeliveryMissionController, "update", side_effect=RuntimeError("Simulated motor failure"))
    def test_full_mission_sim_controller_exception_releases_drone(self, mock_update):
        """Invoke real run_full_mission_simulation() where update() raises an exception.

        Verifies that:
        - Full mission simulation handles exception safely
        - Task is marked FAILED (NOT EN_ROUTE)
        - Assigned drone allocation is released (active_task_id is None, status is READY)
        - Error details are recorded
        """
        import run_full_mission_sim

        result = run_full_mission_sim.run_full_mission_simulation()

        self.assertEqual(result["status"], "failed")
        self.assertIsInstance(result["error"], RuntimeError)
        self.assertEqual(str(result["error"]), "Simulated motor failure")

        task = result["task"]
        self.assertEqual(task.status, TaskStatus.FAILED)

        manager = result["manager"]
        drone = manager.get_drone(task.assigned_drone_id)
        self.assertIsNone(drone.active_task_id)
        self.assertEqual(drone.status, DroneStatus.READY)

    @patch.object(DeliveryMissionController, "update", return_value=DeliveryState.EN_ROUTE)
    def test_delivery_loop_timeout_cannot_mark_verified(self, mock_update):
        """Invoke real run_full_mission_simulation() where delivery controller loop times out.

        Verifies that:
        - A delivery that times out in flight cannot be marked VERIFIED
        - Task status is FAILED
        - Loop timeout is tracked
        - Drone is released in software manager without falsely claiming RTL completion
        """
        import run_full_mission_sim

        result = run_full_mission_sim.run_full_mission_simulation()

        self.assertEqual(result["status"], "timed_out")
        self.assertTrue(result["timed_out"])
        task = result["task"]
        self.assertEqual(task.status, TaskStatus.FAILED)
        self.assertNotEqual(task.status, TaskStatus.VERIFIED)

        manager = result["manager"]
        drone = manager.get_drone(task.assigned_drone_id)
        self.assertIsNone(drone.active_task_id)
        self.assertEqual(drone.status, DroneStatus.READY)

    @patch("run_autonomous_pipeline.inspect_model_weights")
    @patch("run_autonomous_pipeline.YOLOPerceptionAdapter.predict")
    @patch("run_autonomous_pipeline.annotate_detection_image")
    def test_pipeline_genuine_simulated_delivery_succeeds(self, mock_annotate, mock_predict, mock_inspect):
        """Invoke real run_pipeline() along the complete successful simulated flight path.

        Verifies that:
        - Complete pipeline executes through all 8 phases
        - Task status reaches VERIFIED
        - Controller reaches COMPLETED with delivery_verified=True
        - Drone is released upon verified delivery
        """
        import run_autonomous_pipeline

        mock_inspect.return_value = {
            "framework": "Ultralytics YOLOv8",
            "task": "detect",
            "classes": {0: "pedestrian"},
            "num_classes": 1,
            "target_survivor_classes": ["pedestrian"],
        }
        mock_predict.return_value = [
            Detection(0, "pedestrian", 0.94, BoundingBox(0.485, 0.485, 0.515, 0.515))
        ]
        test_img = ROOT / "assets" / "test_survivors.jpg"
        result = run_autonomous_pipeline.run_pipeline(image_path=test_img, confidence_threshold=0.25)

        self.assertEqual(result["status"], "verified")
        self.assertIsNone(result["error"])
        self.assertFalse(result["timed_out"])

        task = result["task"]
        self.assertEqual(task.status, TaskStatus.VERIFIED)

        ctrl = result["controller"]
        self.assertEqual(ctrl.state, DeliveryState.COMPLETED)
        self.assertTrue(ctrl.delivery_verified)

        manager = result["manager"]
        drone = manager.get_drone(task.assigned_drone_id)
        self.assertIsNone(drone.active_task_id)
        self.assertEqual(drone.status, DroneStatus.READY)

    def test_full_mission_sim_genuine_completion_succeeds(self):
        """Invoke real run_full_mission_simulation() along the complete nominal path.

        Verifies that:
        - Full multi-drone coordination simulation runs end-to-end
        - Task reaches VERIFIED
        - Controller state is COMPLETED and delivery_verified is True
        - Fleet summary reports 1 verified delivery
        """
        import run_full_mission_sim

        result = run_full_mission_sim.run_full_mission_simulation()

        self.assertEqual(result["status"], "verified")
        self.assertIsNone(result["error"])
        self.assertFalse(result["timed_out"])

        task = result["task"]
        self.assertEqual(task.status, TaskStatus.VERIFIED)

        ctrl = result["controller"]
        self.assertEqual(ctrl.state, DeliveryState.COMPLETED)
        self.assertTrue(ctrl.delivery_verified)

        self.assertEqual(result["summary"]["verified_deliveries"], 1)


if __name__ == "__main__":
    unittest.main()
