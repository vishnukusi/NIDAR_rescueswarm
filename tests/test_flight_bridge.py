import sys
from pathlib import Path
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.approach_controller import VisualServoCommand
from rescue_swarm_core.flight_bridge import (
    AutopilotType,
    FlightBridgeConfig,
    MAVLinkFlightBridge,
    VehicleConnectionState,
)
from rescue_swarm_core.fleet_flight_bridge import FleetFlightBridgeManager
from rescue_swarm_core.safety_manager import SafetyAction, SafetyAlert, SafetyLevel


class FlightBridgeTests(unittest.TestCase):
    def setUp(self):
        self.config = FlightBridgeConfig(
            drone_id="S1",
            autopilot_type=AutopilotType.MOCK,
            connection_string="mock:127.0.0.1:14550",
            target_system=1,
            target_component=1,
            heartbeat_timeout_s=1.5,
            telemetry_stale_timeout_s=1.0,
        )
        self.bridge = MAVLinkFlightBridge(self.config)

    def tearDown(self):
        self.bridge.disconnect()

    def test_connect_and_heartbeat(self):
        connected = self.bridge.connect(timeout_s=2.0)
        self.assertTrue(connected)
        self.assertTrue(self.bridge.is_connected)
        self.assertEqual(self.bridge.connection_state, VehicleConnectionState.CONNECTED)

    def test_arm_and_takeoff(self):
        self.bridge.connect(timeout_s=2.0)
        # Arm
        self.assertTrue(self.bridge.arm())
        self.bridge.update_telemetry()
        self.assertTrue(self.bridge.telemetry.armed)

        # Set Guided
        self.assertTrue(self.bridge.set_guided_mode())
        self.bridge.update_telemetry()
        self.assertEqual(self.bridge.telemetry.mode, "GUIDED")

        # Takeoff to 15m
        self.assertTrue(self.bridge.takeoff(15.0))
        self.bridge.update_telemetry()
        self.assertEqual(self.bridge.telemetry.altitude_agl_m, 15.0)

    def test_visual_servo_command_conversion(self):
        self.bridge.connect(timeout_s=2.0)
        self.bridge.arm()
        self.bridge.update_telemetry()

        # Command visual servo translation: North +0.8m/s, East -0.4m/s, hold yaw 0.78 rad
        cmd = VisualServoCommand(
            error_x=-40.0,
            error_y=80.0,
            body_forward_mps=-0.64,
            body_right_mps=-0.32,
            local_ned_north_mps=0.80,
            local_ned_east_mps=-0.40,
            action_label="FORWARD + LEFT",
            is_centered=False,
            hold_yaw_rad=0.7854,
        )
        sent = self.bridge.apply_visual_servo_command(cmd)
        self.assertTrue(sent)
        self.bridge.update_telemetry()

        self.assertEqual(self.bridge.telemetry.velocity_north_mps, 0.80)
        self.assertEqual(self.bridge.telemetry.velocity_east_mps, -0.40)
        self.assertAlmostEqual(self.bridge.telemetry.yaw_rad, 0.7854, places=3)

    def test_safety_overrides(self):
        self.bridge.connect(timeout_s=2.0)

        # 1. HOLD sends zero velocity
        self.assertTrue(self.bridge.send_safety_override(SafetyAction.HOLD))
        self.bridge.update_telemetry()
        self.assertEqual(self.bridge.telemetry.velocity_north_mps, 0.0)
        self.assertEqual(self.bridge.telemetry.velocity_east_mps, 0.0)

        # 2. RTL sets RTL mode and zero velocity
        self.assertTrue(self.bridge.send_safety_override(SafetyAction.RTL))
        self.bridge.update_telemetry()
        self.assertEqual(self.bridge.telemetry.mode, "RTL")
        self.assertEqual(self.bridge.telemetry.velocity_north_mps, 0.0)

        # 3. EMERGENCY_LAND sets LAND mode
        self.assertTrue(self.bridge.send_safety_override(SafetyAction.EMERGENCY_LAND))
        self.bridge.update_telemetry()
        self.assertEqual(self.bridge.telemetry.mode, "LAND")

    def test_telemetry_freshness_and_timeout(self):
        self.bridge.connect(timeout_s=2.0)
        self.bridge.update_telemetry()
        self.assertTrue(self.bridge.is_telemetry_fresh())

        # Simulate time jump exceeding freshness timeout
        future_time = time.time() + 5.0
        self.assertFalse(self.bridge.is_telemetry_fresh(current_time=future_time))
        self.bridge.update_telemetry(current_time=future_time)
        self.assertEqual(self.bridge.connection_state, VehicleConnectionState.HEARTBEAT_LOST)

    def test_multi_vehicle_isolation(self):
        bridge_s2 = MAVLinkFlightBridge(
            FlightBridgeConfig(
                drone_id="S2",
                autopilot_type=AutopilotType.MOCK,
                connection_string="mock:127.0.0.1:14551",
                target_system=2,
            )
        )
        self.bridge.connect(timeout_s=2.0)
        bridge_s2.connect(timeout_s=2.0)

        self.assertTrue(self.bridge.is_connected)
        self.assertTrue(bridge_s2.is_connected)

        # Command S1 North=1.0, East=0.0
        self.bridge.send_velocity_ned(1.0, 0.0)
        # Command S2 North=0.0, East=2.0
        bridge_s2.send_velocity_ned(0.0, 2.0)

        self.bridge.update_telemetry()
        bridge_s2.update_telemetry()

        # Verify vehicle instances are isolated without cross-talk
        self.assertEqual(self.bridge.telemetry.velocity_north_mps, 1.0)
        self.assertEqual(self.bridge.telemetry.velocity_east_mps, 0.0)

        self.assertEqual(bridge_s2.telemetry.velocity_north_mps, 0.0)
        self.assertEqual(bridge_s2.telemetry.velocity_east_mps, 2.0)

        bridge_s2.disconnect()

    def test_to_drone_telemetry_conversion(self):
        self.bridge.connect(timeout_s=2.0)
        self.bridge.takeoff(30.0)
        self.bridge.update_telemetry()

        telem = self.bridge.to_drone_telemetry()
        self.assertEqual(telem.drone_id, "S1")
        self.assertEqual(telem.altitude_agl_m, 30.0)
        self.assertEqual(telem.position.latitude, 12.972000)
        self.assertEqual(telem.position.longitude, 77.594500)

    def test_fleet_flight_bridge_manager(self):
        manager = FleetFlightBridgeManager(
            autopilot_type=AutopilotType.MOCK,
            connection_scheme="mock",
        )
        # Verify all 6 drones are configured
        self.assertEqual(len(manager.bridges), 6)
        for drone_id in ["S1", "S2", "S3", "D1", "D2", "D3"]:
            self.assertIn(drone_id, manager.bridges)

        # Connect all
        results = manager.connect_all()
        for drone_id, connected in results.items():
            self.assertTrue(connected)

        # Enforce safety preemption
        alerts = [
            SafetyAlert(
                level=SafetyLevel.CRITICAL,
                action=SafetyAction.RTL,
                drone_id="S2",
                message="Geofence breach detected",
            ),
            SafetyAlert(
                level=SafetyLevel.CRITICAL,
                action=SafetyAction.EMERGENCY_LAND,
                drone_id="D1",
                message="Critical low battery",
            ),
        ]
        enforced = manager.enforce_safety_alerts(alerts)
        self.assertEqual(enforced["S2"], SafetyAction.RTL)
        self.assertEqual(enforced["D1"], SafetyAction.EMERGENCY_LAND)

        manager.update_all_telemetry()
        self.assertEqual(manager.get_bridge("S2").telemetry.mode, "RTL")
        self.assertEqual(manager.get_bridge("D1").telemetry.mode, "LAND")

        manager.disconnect_all()

    def test_real_mode_refuses_success_without_mavlink_response(self):
        # Configure bridge with real MAVLink UDP scheme to an unserved endpoint
        real_config = FlightBridgeConfig(
            drone_id="S1",
            autopilot_type=AutopilotType.ARDUPILOT,
            connection_string="udp:127.0.0.1:19999",
        )
        real_bridge = MAVLinkFlightBridge(real_config)

        self.assertFalse(real_bridge.is_mock_backend)
        self.assertEqual(real_bridge.backend_name, "REAL_MAVLINK")

        # Connection MUST fail without silent fallback
        connected = real_bridge.connect(timeout_s=0.2)
        self.assertFalse(connected)
        self.assertFalse(real_bridge.is_connected)
        self.assertEqual(real_bridge.connection_state, VehicleConnectionState.DISCONNECTED)
        self.assertIsNotNone(real_bridge.last_error)

        # Preflight checks MUST fail
        passed, logs = real_bridge.run_preflight_checks()
        self.assertFalse(passed)
        self.assertTrue(any("FAIL" in log for log in logs))

        # Flight commands MUST refuse to claim success while disconnected
        self.assertFalse(real_bridge.set_guided_mode())
        self.assertFalse(real_bridge.arm())
        self.assertFalse(real_bridge.takeoff(10.0))
        self.assertFalse(real_bridge.send_velocity_ned(1.0, 0.0))
        self.assertFalse(real_bridge.send_safety_override(SafetyAction.RTL))

        real_bridge.disconnect()

    def test_preflight_checks_pass_in_mock_mode(self):
        self.bridge.connect()
        passed, logs = self.bridge.run_preflight_checks()
        self.assertTrue(passed)
        self.assertTrue(all("PASS" in log for log in logs))

    def test_system_id_mismatch_fails_preflight(self):
        self.bridge.connect()
        # Simulate connecting to drone S2 (sysid=2) when bridge expected S1 (sysid=1)
        self.bridge.actual_system_id = 2
        passed, logs = self.bridge.run_preflight_checks()
        self.assertFalse(passed)
        self.assertTrue(any("FAIL: Vehicle System ID mismatch" in log for log in logs))

    def test_arducopter_mode_decoding_accuracy(self):
        from pymavlink import mavutil
        # Verify custom_mode mappings directly matching ArduCopter SITL definitions
        mode_map = mavutil.mode_mapping_acm
        self.assertEqual(mode_map.get(0), "STABILIZE")
        self.assertEqual(mode_map.get(4), "GUIDED")
        self.assertEqual(mode_map.get(6), "RTL")
        self.assertEqual(mode_map.get(9), "LAND")
        self.assertEqual(mode_map.get(16), "POSHOLD")

    def test_local_ned_velocity_convention(self):
        self.bridge.connect()
        # Send North=+2.0, East=+1.5, Down=-0.5 (climbing), Yaw=1.57 rad (East facing)
        self.assertTrue(self.bridge.send_velocity_ned(2.0, 1.5, -0.5, yaw_rad=1.5708))
        self.bridge.update_telemetry()
        self.assertEqual(self.bridge.telemetry.velocity_north_mps, 2.0)
        self.assertEqual(self.bridge.telemetry.velocity_east_mps, 1.5)
        self.assertEqual(self.bridge.telemetry.velocity_down_mps, -0.5)
        self.assertAlmostEqual(self.bridge.telemetry.yaw_rad, 1.5708, places=3)
        self.assertAlmostEqual(self.bridge.telemetry.heading_deg, 90.0, places=1)


if __name__ == "__main__":
    unittest.main()
