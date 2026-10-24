import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.approach_controller import (
    ApproachControllerConfig,
    ApproachState,
    SearchDroneApproachController,
)
from rescue_swarm_core.localization import CameraCalibration
from rescue_swarm_core.perception import BoundingBox, Detection
from rescue_swarm_core.safety_manager import SafetyAction


class ApproachControllerTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration(image_width=1280, image_height=720)
        self.config = ApproachControllerConfig(
            deadband_x_px=30,
            deadband_y_px=30,
            kp_forward=0.008,
            kp_right=0.008,
            max_speed_mps=1.0,
            target_loss_timeout_s=3.0,
        )
        self.controller = SearchDroneApproachController(
            drone_id="S1",
            camera=self.camera,
            config=self.config,
        )

    def _make_detection(self, cx: float = 0.5, cy: float = 0.5, conf: float = 0.85) -> Detection:
        # Construct bbox around (cx, cy)
        w = 0.05
        h = 0.15
        return Detection(
            class_id=0,
            class_name="pedestrian",
            confidence=conf,
            bbox=BoundingBox(
                xmin=max(0.0, cx - w),
                ymin=max(0.0, cy - h),
                xmax=min(1.0, cx + w),
                ymax=min(1.0, cy + h),
            ),
        )

    def test_initial_state_is_search_patrol(self):
        self.assertEqual(self.controller.state, ApproachState.SEARCH_PATROL)
        self.assertFalse(self.controller.is_waypoint_execution_paused)

    def test_detection_triggers_approach_and_pauses_waypoints(self):
        # Target located to the right and below center (cx=0.75, cy=0.75)
        det = self._make_detection(cx=0.75, cy=0.75)
        state, cmd = self.controller.update(det, current_yaw_rad=0.0, dt_s=0.1, current_time=100.0)

        self.assertEqual(state, ApproachState.APPROACHING)
        self.assertTrue(self.controller.is_waypoint_execution_paused)
        self.assertFalse(cmd.is_centered)
        # Should command right and backward
        self.assertGreater(cmd.body_right_mps, 0.0)
        self.assertLess(cmd.body_forward_mps, 0.0)

    def test_centered_detection_transitions_to_hovering(self):
        # Target positioned such that head_y (y1 + 0.12*(y2-y1)) aligns with center_y=360
        # For cy=0.61 with h=0.15: y1=0.46*720=331, y2=0.76*720=547 -> head_y = 331 + 25 = 356 (error = -4 px < 30 deadband)
        det = self._make_detection(cx=0.5, cy=0.61)
        state, cmd = self.controller.update(det, current_yaw_rad=0.0, dt_s=0.1, current_time=100.0)

        # Allow EMA smoothing to settle
        for t in range(5):
            state, cmd = self.controller.update(det, current_yaw_rad=0.0, dt_s=0.1, current_time=100.0 + t*0.1)

        self.assertEqual(state, ApproachState.HOVERING)
        self.assertTrue(cmd.is_centered)
        self.assertEqual(cmd.body_forward_mps, 0.0)
        self.assertEqual(cmd.body_right_mps, 0.0)

    def test_target_loss_and_recovery_timeout(self):
        # 1. Target detected
        det = self._make_detection(cx=0.7, cy=0.7)
        self.controller.update(det, current_yaw_rad=0.0, dt_s=0.1, current_time=100.0)
        self.assertEqual(self.controller.state, ApproachState.APPROACHING)

        # 2. Target lost for 1 second (< 3.0s timeout)
        state, cmd = self.controller.update(None, current_yaw_rad=0.0, dt_s=0.1, current_time=101.0)
        self.assertEqual(state, ApproachState.TARGET_LOST)
        self.assertTrue(self.controller.is_waypoint_execution_paused)
        self.assertEqual(cmd.body_forward_mps, 0.0)

        # 3. Target lost for 4 seconds (> 3.0s timeout) -> Resume search patrol
        state, cmd = self.controller.update(None, current_yaw_rad=0.0, dt_s=0.1, current_time=104.5)
        self.assertEqual(state, ApproachState.SEARCH_RESUMED)
        self.assertFalse(self.controller.is_waypoint_execution_paused)

    def test_safety_override_takes_precedence(self):
        det = self._make_detection(cx=0.7, cy=0.7)
        self.controller.update(det, current_yaw_rad=0.0, dt_s=0.1, current_time=100.0)

        # Command emergency RTL override
        self.controller.apply_safety_override(SafetyAction.RTL)
        state, cmd = self.controller.update(det, current_yaw_rad=0.0, dt_s=0.1, current_time=100.5)

        self.assertEqual(state, ApproachState.SAFETY_OVERRIDE)
        self.assertEqual(cmd.body_forward_mps, 0.0)
        self.assertEqual(cmd.body_right_mps, 0.0)

    def test_multi_search_drone_independence(self):
        # Verify controller functions independently for S1, S2, S3
        c_s1 = SearchDroneApproachController(drone_id="S1")
        c_s2 = SearchDroneApproachController(drone_id="S2")
        c_s3 = SearchDroneApproachController(drone_id="S3")

        det1 = self._make_detection(cx=0.8, cy=0.8)
        s1_state, _ = c_s1.update(det1, current_yaw_rad=0.0, dt_s=0.1, current_time=10.0)
        s2_state, _ = c_s2.update(None, current_yaw_rad=0.0, dt_s=0.1, current_time=10.0)
        s3_state, _ = c_s3.update(None, current_yaw_rad=0.0, dt_s=0.1, current_time=10.0)

        self.assertEqual(s1_state, ApproachState.APPROACHING)
        self.assertEqual(s2_state, ApproachState.SEARCH_PATROL)
        self.assertEqual(s3_state, ApproachState.SEARCH_PATROL)
        self.assertTrue(c_s1.is_waypoint_execution_paused)
        self.assertFalse(c_s2.is_waypoint_execution_paused)


if __name__ == "__main__":
    unittest.main()
