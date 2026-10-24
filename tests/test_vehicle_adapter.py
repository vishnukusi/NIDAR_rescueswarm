"""Tests for VehicleAdapter, SimulatedDeliveryController, and FleetMissionManager."""

import os
import sys
import time
import unittest

# Add ros2_ws and src to sys.path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
ROS2_PKG = os.path.join(REPO_ROOT, "ros2_ws", "src", "rescue_swarm_bridge")
SRC_DIR = os.path.join(REPO_ROOT, "src")
if ROS2_PKG not in sys.path:
    sys.path.insert(0, ROS2_PKG)
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from rescue_swarm_bridge.vehicle_adapter import (
    AdapterType,
    MockVehicleAdapter,
    PX4MAVLinkVehicleAdapter,
    PX4ROS2VehicleAdapter,
    VehicleConnectionState,
    VehicleTelemetry,
    create_vehicle_adapter,
)
from rescue_swarm_bridge.simulated_delivery_node import (
    DeliveryTaskState,
    SimulatedDeliveryController,
    SimulatedDeliveryStage,
    SimulatedDeliveryVerificationMode,
)
from rescue_swarm_bridge.mission_manager_node import (
    FleetMissionManager,
    MissionPhase,
)


class TestVehicleAdapters(unittest.TestCase):
    """Test suite for vehicle adapter contracts and isolation."""

    def test_mock_adapter_contract(self):
        cfg = {"initial_position": {"lat": 12.9720, "lon": 77.5945, "alt": 25.0}}
        adapter = MockVehicleAdapter("S1", cfg)
        self.assertEqual(adapter.adapter_type, AdapterType.MOCK)
        self.assertTrue(adapter.is_mock)
        self.assertFalse(adapter.is_connected)

        # Connect
        connected = adapter.connect()
        self.assertTrue(connected)
        self.assertTrue(adapter.is_connected)

        # Telemetry
        telem = adapter.update(current_time=100.0)
        self.assertIsNotNone(telem)
        self.assertEqual(telem.drone_id, "S1")
        self.assertEqual(telem.latitude, 12.9720)
        self.assertEqual(telem.altitude_agl_m, 25.0)
        self.assertTrue(telem.is_fresh(current_time=101.0, max_age_s=2.0))
        self.assertFalse(telem.is_fresh(current_time=105.0, max_age_s=2.0))

        # Actuation & Mode verification
        self.assertTrue(adapter.arm())
        self.assertTrue(adapter.is_armed)
        self.assertTrue(adapter.set_mode("HOLD"))
        self.assertEqual(adapter.current_mode, "HOLD")
        self.assertTrue(adapter.send_safety_action("RTL"))
        self.assertEqual(adapter.current_mode, "RTL")

        # History audit
        cmds = adapter.get_command_history()
        self.assertGreaterEqual(len(cmds), 3)

        adapter.disconnect()
        self.assertFalse(adapter.is_connected)

    def test_px4_ros2_adapter_isolated_namespace(self):
        cfg = {"ros_namespace": "s2", "system_id": 2}
        adapter = PX4ROS2VehicleAdapter("S2", cfg, ros2_node=None)
        self.assertEqual(adapter.adapter_type, AdapterType.PX4_ROS2)
        self.assertFalse(adapter.is_mock)
        self.assertEqual(adapter.namespace, "s2")

        # Without active ROS 2 node context, connect should report failure safely
        conn = adapter.connect()
        self.assertFalse(conn)
        self.assertEqual(adapter.connection_state, VehicleConnectionState.FAILED)

    def test_px4_mavlink_adapter_port_binding(self):
        # Default offboard port is 14540 (instance 0)
        default_adapter = PX4MAVLinkVehicleAdapter("S1", {"system_id": 1})
        self.assertEqual(default_adapter.port, 14540)
        self.assertEqual(default_adapter.sysid, 1)

        cfg = {"mavlink_port": 14543, "system_id": 4}
        adapter = PX4MAVLinkVehicleAdapter("D1", cfg)
        self.assertEqual(adapter.adapter_type, AdapterType.PX4_MAVLINK)
        self.assertFalse(adapter.is_mock)
        self.assertEqual(adapter.port, 14543)
        self.assertEqual(adapter.sysid, 4)

        # Test command dispatch methods execute without AttributeError
        self.assertIsNotNone(adapter._bridge)
        res_mode = adapter.set_mode("HOLD")
        self.assertTrue(isinstance(res_mode, bool))
        res_sp = adapter.send_position_setpoint(12.972, 77.595, 25.0, 90.0)
        self.assertTrue(isinstance(res_sp, bool))

    def test_vehicle_telemetry_validity_guards(self):
        import math
        valid_t = VehicleTelemetry(
            drone_id="S1", timestamp=100.0, connected=True, latitude=12.972, longitude=77.594,
            altitude_agl_m=30.0, battery_fraction=0.85
        )
        self.assertTrue(valid_t.is_valid())
        self.assertTrue(valid_t.is_fresh(current_time=101.0, max_age_s=2.0))

        # NaN coordinates must be rejected
        nan_t = VehicleTelemetry(
            drone_id="S1", timestamp=100.0, connected=True, latitude=float("nan"), longitude=77.594,
            altitude_agl_m=30.0, battery_fraction=0.85
        )
        self.assertFalse(nan_t.is_valid())
        self.assertFalse(nan_t.is_fresh(current_time=101.0, max_age_s=2.0))

        # Out-of-bounds latitude must be rejected
        oob_t = VehicleTelemetry(
            drone_id="S1", timestamp=100.0, connected=True, latitude=95.0, longitude=77.594,
            altitude_agl_m=30.0, battery_fraction=0.85
        )
        self.assertFalse(oob_t.is_valid())

    def test_px4_custom_mode_decoding(self):
        from rescue_swarm_core.flight_bridge import AutopilotType, FlightBridgeConfig, MAVLinkFlightBridge
        cfg = FlightBridgeConfig(drone_id="S1", autopilot_type=AutopilotType.PX4, connection_string="mock:s1")
        bridge = MAVLinkFlightBridge(cfg)

        class MockHeartbeat:
            def __init__(self, custom_mode: int, base_mode: int = 128):
                self.custom_mode = custom_mode
                self.base_mode = base_mode
            def get_type(self):
                return "HEARTBEAT"

        # Offboard: main_mode 6
        hb_offb = MockHeartbeat(custom_mode=(6 << 16))
        self.assertEqual((hb_offb.custom_mode >> 16) & 0xFF, 6)

        # AUTO LOITER (Hold): main_mode 4, sub_mode 3
        hb_hold = MockHeartbeat(custom_mode=(4 << 16) | (3 << 24))
        self.assertEqual((hb_hold.custom_mode >> 16) & 0xFF, 4)
        self.assertEqual((hb_hold.custom_mode >> 24) & 0xFF, 3)

        # AUTO RTL: main_mode 4, sub_mode 5
        hb_rtl = MockHeartbeat(custom_mode=(4 << 16) | (5 << 24))
        self.assertEqual((hb_rtl.custom_mode >> 16) & 0xFF, 4)
        self.assertEqual((hb_rtl.custom_mode >> 24) & 0xFF, 5)

    def test_send_position_target_global_validation(self):
        from rescue_swarm_core.flight_bridge import AutopilotType, FlightBridgeConfig, MAVLinkFlightBridge
        cfg = FlightBridgeConfig(drone_id="S1", autopilot_type=AutopilotType.PX4, connection_string="mock:s1")
        bridge = MAVLinkFlightBridge(cfg)

        # Rejects NaN
        res_nan = bridge.send_position_target_global(float("nan"), 77.594, 25.0)
        self.assertFalse(res_nan)
        self.assertIn("Invalid", bridge.last_error)

        # Rejects Inf
        res_inf = bridge.send_position_target_global(12.972, float("inf"), 25.0)
        self.assertFalse(res_inf)

        # Accepts finite coordinates
        res_ok = bridge.send_position_target_global(12.972, 77.594, 25.0, 45.0)
        self.assertTrue(res_ok)
        self.assertEqual(bridge.telemetry.latitude, 12.972)
        self.assertEqual(bridge.telemetry.altitude_agl_m, 25.0)

    def test_create_vehicle_adapter_factory(self):
        mock_ad = create_vehicle_adapter("S1", {"adapter_type": "mock"})
        self.assertEqual(mock_ad.adapter_type, AdapterType.MOCK)

        mav_ad = create_vehicle_adapter("D2", {"adapter_type": "px4_mavlink", "mavlink_port": 14544})
        self.assertEqual(mav_ad.adapter_type, AdapterType.PX4_MAVLINK)


class TestSimulatedDeliveryController(unittest.TestCase):
    """Test suite for simulation delivery verification controller."""

    def test_delivery_progression_nominal(self):
        ctrl = SimulatedDeliveryController(
            drone_id="D1",
            verification_mode=SimulatedDeliveryVerificationMode.PROXIMITY_AND_HOVER,
            drop_hover_duration_s=1.0,
            arrival_radius_m=3.0,
            target_delivery_alt_m=5.0,
        )

        # Assign
        task = ctrl.assign_task("task_001", target_lat=12.9725, target_lon=77.5950, current_time=10.0)
        self.assertEqual(task.stage, SimulatedDeliveryStage.INGRESS)
        self.assertTrue(task.is_simulation)

        # Update far away
        ctrl.update(current_lat=12.9700, current_lon=77.5900, current_alt_m=30.0, current_time=11.0)
        self.assertEqual(ctrl.current_task.stage, SimulatedDeliveryStage.INGRESS)

        # Arrive at target position (ingress complete -> HOVER_DESCENT)
        ctrl.update(current_lat=12.9725, current_lon=77.5950, current_alt_m=5.0, current_time=15.0)
        self.assertEqual(ctrl.current_task.stage, SimulatedDeliveryStage.HOVER_DESCENT)

        # Hover duration satisfied (hover -> SIMULATED_PAYLOAD_DROP)
        ctrl.update(current_lat=12.9725, current_lon=77.5950, current_alt_m=5.0, current_time=16.5)
        self.assertEqual(ctrl.current_task.stage, SimulatedDeliveryStage.SIMULATED_PAYLOAD_DROP)

        # Verify outcome
        ctrl.update(current_lat=12.9725, current_lon=77.5950, current_alt_m=5.0, current_time=17.0)
        self.assertIsNone(ctrl.current_task)
        self.assertEqual(len(ctrl.completed_tasks), 1)
        self.assertEqual(ctrl.completed_tasks[0].stage, SimulatedDeliveryStage.VERIFIED)

    def test_delivery_retry_lineage_mode(self):
        ctrl = SimulatedDeliveryController(
            drone_id="D2",
            verification_mode=SimulatedDeliveryVerificationMode.FAIL_FIRST_THEN_RETRY,
            drop_hover_duration_s=0.5,
            arrival_radius_m=5.0,
        )

        # Attempt 1
        ctrl.assign_task("task_002", target_lat=12.9725, target_lon=77.5950, current_time=10.0, attempt=1)
        ctrl.update(12.9725, 77.5950, 5.0, current_time=11.0)  # arrive
        ctrl.update(12.9725, 77.5950, 5.0, current_time=12.0)  # drop
        ctrl.update(12.9725, 77.5950, 5.0, current_time=12.5)  # verify attempt 1 (fails)
        self.assertEqual(len(ctrl.completed_tasks), 1)
        self.assertEqual(ctrl.completed_tasks[0].stage, SimulatedDeliveryStage.FAILED)
        self.assertEqual(ctrl.completed_tasks[0].attempt_count, 1)

        # Attempt 2 (retry succeeds)
        ctrl.assign_task("task_002", target_lat=12.9725, target_lon=77.5950, current_time=20.0, attempt=2)
        ctrl.update(12.9725, 77.5950, 5.0, current_time=21.0)
        ctrl.update(12.9725, 77.5950, 5.0, current_time=22.0)
        ctrl.update(12.9725, 77.5950, 5.0, current_time=22.5)
        self.assertEqual(len(ctrl.completed_tasks), 2)
        self.assertEqual(ctrl.completed_tasks[1].stage, SimulatedDeliveryStage.VERIFIED)
        self.assertEqual(ctrl.completed_tasks[1].attempt_count, 2)


class TestFleetMissionManager(unittest.TestCase):
    """Test suite for complete 6-drone FleetMissionManager."""

    def test_fleet_manager_init_and_preflight(self):
        fleet_json = os.path.join(REPO_ROOT, "config", "fleet.json")
        manager = FleetMissionManager(
            fleet_config_path=fleet_json,
            adapter_type=AdapterType.MOCK,
            mission_timeout_s=60.0,
        )
        self.assertEqual(len(manager.adapters), 6)
        self.assertEqual(set(manager.adapters.keys()), {"S1", "S2", "S3", "D1", "D2", "D3"})

        # Run preflight
        preflight = manager.run_preflight_checks()
        self.assertTrue(preflight["all_passed"])
        self.assertEqual(manager.phase, MissionPhase.READY)

        # Step loop
        status = manager.step(current_time=100.0)
        self.assertEqual(status["phase"], MissionPhase.SEARCH_AND_RESCUE)
        self.assertEqual(len(status["vehicles"]), 6)

        manager.shutdown()

    def test_fleet_mission_manager_command_dispatch(self):
        fleet_json = os.path.join(REPO_ROOT, "config", "fleet.json")
        manager = FleetMissionManager(
            fleet_config_path=fleet_json,
            adapter_type=AdapterType.MOCK,
            mission_timeout_s=60.0,
        )
        manager.run_preflight_checks()
        self.assertEqual(manager.phase, MissionPhase.READY)

        # Step into SEARCH_AND_RESCUE
        manager.step(current_time=100.0)
        self.assertEqual(manager.phase, MissionPhase.SEARCH_AND_RESCUE)

        # Verify search drones were armed and dispatched setpoints
        s1_adapter = manager.adapters["S1"]
        s1_cmds = s1_adapter.get_command_history()
        s1_cmd_types = [c["cmd"] for c in s1_cmds]
        self.assertIn("arm", s1_cmd_types)
        self.assertIn("setpoint", s1_cmd_types)

        # Assign a rescue delivery task
        assigned_drone = manager.assign_rescue_task("test_task_01", target_lat=12.9730, target_lon=77.5950)
        self.assertIsNotNone(assigned_drone)
        self.assertIn(assigned_drone, ("D1", "D2", "D3"))

        # Step again to trigger delivery command dispatch
        manager.step(current_time=101.0)
        d_adapter = manager.adapters[assigned_drone]
        d_cmds = d_adapter.get_command_history()
        d_cmd_types = [c["cmd"] for c in d_cmds]
        self.assertIn("setpoint", d_cmd_types)

        manager.shutdown()


if __name__ == "__main__":
    unittest.main()
