import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.delivery import (
    DeliveryMissionController,
    DeliveryProfile,
    DeliveryState,
    DeliveryVerificationMethod,
)
from rescue_swarm_core.models import GeoPoint, RescueTask, TaskStatus


class DeliveryControllerTests(unittest.TestCase):
    def setUp(self):
        self.home = GeoPoint(latitude=12.9710, longitude=77.5940, altitude_m=0.0)
        self.survivor_loc = GeoPoint(latitude=12.9715, longitude=77.5945, altitude_m=0.0)
        self.task = RescueTask(
            task_id="task_001",
            confidence=0.92,
            image_x=0.5,
            image_y=0.5,
            detected_by="S1",
            status=TaskStatus.ASSIGNED,
            location=self.survivor_loc,
            assigned_drone_id="D1",
        )
        self.profile = DeliveryProfile(
            transit_altitude_m=20.0,
            release_altitude_m=8.0,
            horizontal_speed_mps=10.0,
            vertical_speed_mps=4.0,
            drop_tolerance_radius_m=3.0,
            release_hover_time_s=1.0,
            verification_time_s=1.0,
        )
        self.controller = DeliveryMissionController(
            drone_id="D1",
            task=self.task,
            home_location=self.home,
            profile=self.profile,
        )

    def test_delivery_mission_lifecycle(self):
        # 1. Standby
        self.assertEqual(self.controller.state, DeliveryState.STANDBY)
        self.controller.start_mission()
        self.assertEqual(self.controller.state, DeliveryState.TAKEOFF)
        self.assertEqual(self.task.status, TaskStatus.EN_ROUTE)

        # 2. Takeoff climb
        # Needs 20.0 / 4.0 = 5.0 seconds to climb
        self.controller.update(5.0)
        self.assertEqual(self.controller.state, DeliveryState.EN_ROUTE)
        self.assertEqual(self.controller.altitude_agl_m, 20.0)

        # 3. Transit to survivor approach
        # Distance is ~77 meters. At 10 m/s, ~8 seconds
        for _ in range(12):
            self.controller.update(1.0)
            if self.controller.state in (DeliveryState.APPROACH, DeliveryState.DESCENT_TO_DROP):
                break

        self.assertIn(self.controller.state, (DeliveryState.APPROACH, DeliveryState.DESCENT_TO_DROP))

        # 4. Step until DESCENT_TO_DROP then PAYLOAD_RELEASE
        for _ in range(10):
            self.controller.update(1.0)
            if self.controller.state == DeliveryState.PAYLOAD_RELEASE:
                break
        self.assertEqual(self.controller.state, DeliveryState.PAYLOAD_RELEASE)
        self.assertEqual(self.controller.altitude_agl_m, 8.0)

        # 5. Payload release
        self.controller.update(1.1)
        self.assertTrue(self.controller.payload_released)
        self.assertEqual(self.task.status, TaskStatus.RELEASED)
        self.assertEqual(self.controller.state, DeliveryState.VERIFY_DROP)

        # 6. Drop verification
        self.controller.update(1.1)
        self.assertTrue(self.controller.delivery_verified)
        self.assertEqual(self.task.status, TaskStatus.VERIFIED)
        self.assertEqual(self.controller.state, DeliveryState.ASCENT)

        # 7. Ascent then Return Home
        self.controller.update(3.0)
        self.assertEqual(self.controller.state, DeliveryState.RETURNING_HOME)

        # 8. Complete RTL
        for _ in range(15):
            self.controller.update(1.0)
            if self.controller.state == DeliveryState.COMPLETED:
                break
        self.assertEqual(self.controller.state, DeliveryState.COMPLETED)
        self.assertEqual(self.controller.altitude_agl_m, 0.0)

    def test_abort_triggers_immediate_rtl(self):
        self.controller.start_mission()
        self.controller.update(2.0)
        self.controller.abort()
        self.assertEqual(self.controller.state, DeliveryState.RETURNING_HOME)

    def test_sensor_event_verification(self):
        controller = DeliveryMissionController(
            drone_id="D2",
            task=self.task,
            home_location=self.home,
            profile=self.profile,
            verification_method=DeliveryVerificationMethod.GROUND_CONTACT_SENSOR,
        )
        controller.start_mission()
        controller.update(5.0)  # Climb
        for _ in range(12):
            controller.update(1.0)
            if controller.state in (DeliveryState.APPROACH, DeliveryState.DESCENT_TO_DROP):
                break
        for _ in range(10):
            controller.update(1.0)
            if controller.state == DeliveryState.PAYLOAD_RELEASE:
                break
        controller.update(1.1)  # Release payload -> VERIFY_DROP
        self.assertEqual(controller.state, DeliveryState.VERIFY_DROP)

        # In VERIFY_DROP, elapsed timer does NOT prematurely verify without sensor
        controller.update(2.0)
        self.assertEqual(controller.state, DeliveryState.VERIFY_DROP)
        self.assertFalse(controller.delivery_verified)

        # Signal real sensor event
        controller.confirm_ground_delivery("ground_switch_gpio", "Impact detected at 0.5 m/s")
        self.assertTrue(controller.delivery_verified)
        self.assertEqual(controller.verification_source, "ground_switch_gpio")

        controller.update(0.1)
        self.assertEqual(controller.state, DeliveryState.ASCENT)

    def test_sensor_event_timeout(self):
        profile = DeliveryProfile(
            transit_altitude_m=20.0,
            release_altitude_m=8.0,
            horizontal_speed_mps=10.0,
            vertical_speed_mps=4.0,
            release_hover_time_s=1.0,
            verification_time_s=1.0,
            verification_timeout_s=5.0,
        )
        controller = DeliveryMissionController(
            drone_id="D3",
            task=self.task,
            home_location=self.home,
            profile=profile,
            verification_method=DeliveryVerificationMethod.SIMULATION_EVENT,
        )
        controller.start_mission()
        controller.update(5.0)
        for _ in range(25):
            controller.update(1.0)
            if controller.state == DeliveryState.VERIFY_DROP:
                break
        self.assertEqual(controller.state, DeliveryState.VERIFY_DROP)

        # Wait past verification_timeout_s
        controller.update(6.0)
        self.assertEqual(controller.state, DeliveryState.ASCENT)
        self.assertFalse(controller.delivery_verified)
        self.assertIn("timed out", controller.verification_details)


if __name__ == "__main__":
    unittest.main()
