import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.coverage_planner import SearchArea
from rescue_swarm_core.localization import DroneTelemetry
from rescue_swarm_core.models import GeoPoint
from rescue_swarm_core.safety_manager import (
    BatteryFailsafeConfig,
    GeofenceConfig,
    SafetyAction,
    SafetyLevel,
    SafetyManager,
)


class SafetyManagerTests(unittest.TestCase):
    def setUp(self):
        self.boundary = SearchArea(
            min_lat=12.9700,
            min_lon=77.5900,
            max_lat=12.9750,
            max_lon=77.5950,
        )
        self.geofence = GeofenceConfig(
            boundary=self.boundary,
            min_altitude_agl_m=5.0,
            max_altitude_agl_m=50.0,
        )
        self.battery_config = BatteryFailsafeConfig(
            warning_fraction=0.30,
            rtl_fraction=0.20,
            land_critical_fraction=0.10,
        )
        self.manager = SafetyManager(
            geofence=self.geofence,
            battery_config=self.battery_config,
            min_separation_m=5.0,
            heartbeat_timeout_s=3.0,
        )

    def _make_telemetry(self, drone_id: str, lat: float = 12.9720, lon: float = 77.5920, alt: float = 25.0) -> DroneTelemetry:
        return DroneTelemetry(
            drone_id=drone_id,
            position=GeoPoint(latitude=lat, longitude=lon, altitude_m=alt),
            altitude_agl_m=alt,
            heading_deg=0.0,
        )

    def test_battery_failsafe_hierarchy(self):
        telem = self._make_telemetry("D1")

        # 1. Normal battery
        alerts = self.manager.check_telemetry(telem, battery_fraction=0.75)
        self.assertEqual(len(alerts), 0)

        # 2. Low battery warning (<= 0.30)
        alerts = self.manager.check_telemetry(telem, battery_fraction=0.28)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].level, SafetyLevel.WARNING)
        self.assertEqual(alerts[0].action, SafetyAction.WARN)

        # 3. RTL threshold (<= 0.20)
        alerts = self.manager.check_telemetry(telem, battery_fraction=0.18)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].level, SafetyLevel.CRITICAL)
        self.assertEqual(alerts[0].action, SafetyAction.RTL)

        # 4. Emergency land (<= 0.10)
        alerts = self.manager.check_telemetry(telem, battery_fraction=0.08)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].level, SafetyLevel.EMERGENCY)
        self.assertEqual(alerts[0].action, SafetyAction.EMERGENCY_LAND)

    def test_geofence_breach_detection(self):
        # Drone breaches ceiling
        telem_high = self._make_telemetry("S1", alt=55.0)
        alerts = self.manager.check_telemetry(telem_high, battery_fraction=0.90)
        self.assertTrue(any(a.action == SafetyAction.HOLD for a in alerts))

        # Drone breaches floor
        telem_low = self._make_telemetry("S1", alt=3.0)
        alerts = self.manager.check_telemetry(telem_low, battery_fraction=0.90)
        self.assertTrue(any(a.action == SafetyAction.WARN for a in alerts))

        # Drone breaches lateral boundary
        telem_out = self._make_telemetry("S1", lat=12.9800, alt=25.0)
        alerts = self.manager.check_telemetry(telem_out, battery_fraction=0.90)
        self.assertTrue(any(a.action == SafetyAction.RTL for a in alerts))

    def test_proximity_collision_alert(self):
        # Two drones positioned 2 meters apart (< 5.0m min separation)
        t1 = self._make_telemetry("S1", lat=12.972000, lon=77.592000, alt=25.0)
        t2 = self._make_telemetry("S2", lat=12.972015, lon=77.592000, alt=25.0)

        telemetries = {"S1": t1, "S2": t2}
        alerts = self.manager.check_fleet_proximity(telemetries)
        self.assertEqual(len(alerts), 2)
        self.assertEqual(alerts[0].level, SafetyLevel.CRITICAL)
        self.assertEqual(alerts[0].action, SafetyAction.HOLD)

    def test_heartbeat_timeout(self):
        self.manager.record_heartbeat("S1", current_time=100.0)
        # 2 seconds later -> OK
        alerts = self.manager.check_telemetry_timeouts(["S1"], current_time=102.0)
        self.assertEqual(len(alerts), 0)

        # 4 seconds later -> Timeout (> 3.0s)
        alerts = self.manager.check_telemetry_timeouts(["S1"], current_time=104.0)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].action, SafetyAction.RTL)

    def test_persistent_condition_does_not_create_unbounded_duplicate_incident_records(self):
        """A persistent violation across 10 ticks creates exactly 1 incident record with occurrence_count=10."""
        telem_high = self._make_telemetry("S1", alt=55.0)
        for tick in range(10):
            alerts = self.manager.check_telemetry(telem_high, battery_fraction=0.90, current_time=10.0 + tick)
            self.assertEqual(len(alerts), 1)
            self.assertEqual(alerts[0].action, SafetyAction.HOLD)

        # Exactly 1 incident in log, occurrence_count tracked accurately
        incidents = self.manager.get_all_incidents()
        self.assertEqual(len(incidents), 1)
        inc = incidents[0]
        self.assertEqual(inc.drone_id, "S1")
        self.assertEqual(inc.condition_type, "geofence_ceiling")
        self.assertEqual(inc.occurrence_count, 10)
        self.assertEqual(inc.start_time, 10.0)
        self.assertEqual(inc.last_seen_time, 19.0)
        self.assertFalse(inc.resolved)

        # Alert history deduplicates identical persistent ticks
        self.assertEqual(len(self.manager.alert_history), 1)

    def test_newly_occurring_incident_is_recorded(self):
        """Distinct conditions or different drones produce distinct incident records."""
        telem_s1 = self._make_telemetry("S1", alt=55.0)
        telem_s2 = self._make_telemetry("S2", alt=55.0)

        self.manager.check_telemetry(telem_s1, battery_fraction=0.90, current_time=10.0)
        self.manager.check_telemetry(telem_s2, battery_fraction=0.90, current_time=10.0)

        incidents = self.manager.get_all_incidents()
        self.assertEqual(len(incidents), 2)
        drone_ids = {inc.drone_id for inc in incidents}
        self.assertEqual(drone_ids, {"S1", "S2"})

    def test_severity_escalation_is_recorded(self):
        """Battery drain from WARNING (0.28) -> RTL (0.18) -> EMERGENCY_LAND (0.08) records escalations."""
        telem = self._make_telemetry("D1")

        # 1. Low battery warning (WARNING)
        a1 = self.manager.check_telemetry(telem, battery_fraction=0.28, current_time=10.0)
        self.assertEqual(len(a1), 1)
        self.assertEqual(a1[0].level, SafetyLevel.WARNING)

        # 2. RTL threshold triggered (CRITICAL)
        a2 = self.manager.check_telemetry(telem, battery_fraction=0.18, current_time=20.0)
        self.assertEqual(len(a2), 1)
        self.assertEqual(a2[0].level, SafetyLevel.CRITICAL)

        # 3. Emergency land triggered (EMERGENCY)
        a3 = self.manager.check_telemetry(telem, battery_fraction=0.08, current_time=30.0)
        self.assertEqual(len(a3), 1)
        self.assertEqual(a3[0].level, SafetyLevel.EMERGENCY)

        incidents = self.manager.get_all_incidents()
        self.assertEqual(len(incidents), 1)
        inc = incidents[0]
        self.assertTrue(inc.severity_escalated)
        self.assertEqual(inc.level, SafetyLevel.EMERGENCY)
        self.assertEqual(inc.action, SafetyAction.EMERGENCY_LAND)
        self.assertEqual(len(inc.escalations), 2)
        self.assertEqual(inc.escalations[0]["to_level"], SafetyLevel.CRITICAL.value)
        self.assertEqual(inc.escalations[1]["to_level"], SafetyLevel.EMERGENCY.value)

        # Alert history contains initial alert plus both escalations
        self.assertEqual(len(self.manager.alert_history), 3)

    def test_clearing_and_retriggering_condition_produces_new_incident(self):
        """When an incident condition clears and later recurs, a distinct new incident is logged."""
        telem_breach = self._make_telemetry("S1", alt=55.0)
        telem_nominal = self._make_telemetry("S1", alt=25.0)

        # Step 1: Initial breach at t=10.0
        self.manager.check_telemetry(telem_breach, battery_fraction=0.90, current_time=10.0)
        self.assertEqual(len(self.manager.get_active_incidents()), 1)

        # Step 2: Cleared at t=15.0
        self.manager.check_telemetry(telem_nominal, battery_fraction=0.90, current_time=15.0)
        self.assertEqual(len(self.manager.get_active_incidents()), 0)
        self.assertTrue(self.manager.incidents[0].resolved)
        self.assertEqual(self.manager.incidents[0].end_time, 15.0)

        # Step 3: Re-triggered breach at t=25.0
        self.manager.check_telemetry(telem_breach, battery_fraction=0.90, current_time=25.0)
        self.assertEqual(len(self.manager.get_all_incidents()), 2)
        self.assertEqual(len(self.manager.get_active_incidents()), 1)
        inc1, inc2 = self.manager.get_all_incidents()
        self.assertTrue(inc1.resolved)
        self.assertFalse(inc2.resolved)
        self.assertEqual(inc2.start_time, 25.0)

    def test_genuine_communications_loss_remains_detectable(self):
        """Watchdog correctly identifies silent drones while active drones remain alert-free."""
        self.manager.record_heartbeat("S1", current_time=10.0)
        self.manager.record_heartbeat("S2", current_time=10.0)

        # S1 keeps heartbeating, S2 goes silent
        self.manager.record_heartbeat("S1", current_time=16.0)
        alerts = self.manager.check_telemetry_timeouts(["S1", "S2"], current_time=16.0)

        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].drone_id, "S2")
        self.assertEqual(alerts[0].action, SafetyAction.RTL)
        self.assertIn("Telemetry lost for S2", alerts[0].message)

    def test_safety_actions_not_lost_when_notification_deduplication_applied(self):
        """Controllers receive safety actions on EVERY tick despite incident record deduplication."""
        telem_breach = self._make_telemetry("S1", alt=58.0)
        for tick in range(5):
            alerts = self.manager.check_telemetry(telem_breach, battery_fraction=0.90, current_time=50.0 + tick)
            # Controller MUST get action every single tick to enforce safety override
            self.assertEqual(len(alerts), 1)
    def test_audit_a_telemetry_timeout_lifecycle_dedup_restore_retrigger(self):
        """1. Persistent timeout creates 1 incident with updating occurrence count.
        2. Fresh telemetry restores heartbeat and resolves the incident.
        3. Subsequent timeout creates a new incident ID.
        8. Resolved condition is not counted as active.
        """
        # Step 1: Initial heartbeat at t=10.0
        self.manager.record_heartbeat("S1", current_time=10.0)

        # Timeout occurs at t=14.0 (> 3.0s timeout), checked at 14.0, 15.0, 16.0
        for tick in [14.0, 15.0, 16.0]:
            alerts = self.manager.check_telemetry_timeouts(["S1"], current_time=tick)
            self.assertEqual(len(alerts), 1)
            self.assertEqual(alerts[0].action, SafetyAction.RTL)

        incidents = self.manager.get_all_incidents()
        self.assertEqual(len(incidents), 1)
        inc1 = incidents[0]
        self.assertEqual(inc1.incident_id, "INC-001")
        self.assertEqual(inc1.drone_id, "S1")
        self.assertEqual(inc1.condition_type, "telemetry_timeout")
        self.assertEqual(inc1.occurrence_count, 3)
        self.assertFalse(inc1.resolved)
        self.assertEqual(len(self.manager.get_active_incidents()), 1)

        # Step 2: Fresh telemetry arrives at t=20.0 -> resolves incident
        accepted = self.manager.record_heartbeat("S1", current_time=20.0)
        self.assertTrue(accepted)
        self.assertTrue(inc1.resolved)
        self.assertEqual(inc1.end_time, 20.0)
        # Resolved condition is not counted as active
        self.assertEqual(len(self.manager.get_active_incidents()), 0)

        # Step 3: Subsequent timeout at t=25.0 (> 3.0s after t=20.0)
        alerts_new = self.manager.check_telemetry_timeouts(["S1"], current_time=25.0)
        self.assertEqual(len(alerts_new), 1)
        self.assertEqual(len(self.manager.get_all_incidents()), 2)
        inc1_after, inc2 = self.manager.get_all_incidents()
        self.assertEqual(inc1_after.incident_id, "INC-001")
        self.assertTrue(inc1_after.resolved)
        self.assertEqual(inc2.incident_id, "INC-002")
        self.assertFalse(inc2.resolved)
        self.assertEqual(inc2.start_time, 25.0)
        self.assertEqual(len(self.manager.get_active_incidents()), 1)

    def test_audit_a_false_recovery_prevention(self):
        """Telemetry only counts as fresh when genuinely newer than previous heartbeat."""
        self.manager.record_heartbeat("S1", current_time=50.0)

        # Trigger timeout at t=55.0
        self.manager.check_telemetry_timeouts(["S1"], current_time=55.0)
        self.assertEqual(len(self.manager.get_active_incidents()), 1)
        inc = self.manager.get_active_incidents()[0]
        self.assertFalse(inc.resolved)

        # Stale/duplicate heartbeat with t=48.0 (older than 50.0) or t=50.0 (duplicate)
        rejected_old = self.manager.record_heartbeat("S1", current_time=48.0)
        self.assertFalse(rejected_old)
        rejected_dup = self.manager.record_heartbeat("S1", current_time=50.0)
        self.assertFalse(rejected_dup)

        # Must still be active! No false recovery allowed
        self.assertEqual(len(self.manager.get_active_incidents()), 1)
        self.assertFalse(inc.resolved)

        # Genuinely fresh update with t=56.0 (> 50.0)
        accepted = self.manager.record_heartbeat("S1", current_time=56.0)
        self.assertTrue(accepted)
        self.assertTrue(inc.resolved)
        self.assertEqual(inc.end_time, 56.0)
        self.assertEqual(len(self.manager.get_active_incidents()), 0)

    def test_audit_a_severity_escalation_preserves_start_time(self):
        """Escalation updates severity and logs escalation records while preserving original start_time."""
        telem = self._make_telemetry("D1")
        self.manager.check_telemetry(telem, battery_fraction=0.28, current_time=10.0)
        self.manager.check_telemetry(telem, battery_fraction=0.18, current_time=15.0)
        self.manager.check_telemetry(telem, battery_fraction=0.08, current_time=20.0)

        inc = self.manager.get_all_incidents()[0]
        self.assertEqual(inc.start_time, 10.0)  # Original start time MUST be preserved
        self.assertEqual(inc.last_seen_time, 20.0)
        self.assertEqual(inc.occurrence_count, 3)
        self.assertTrue(inc.severity_escalated)
        self.assertEqual(inc.level, SafetyLevel.EMERGENCY)
        self.assertEqual(len(inc.escalations), 2)

    def test_audit_a_two_drones_same_condition_and_multiple_conditions_on_one_drone(self):
        """Two drones with same condition get separate incidents; different conditions on one drone remain distinct."""
        telem_d1_high = self._make_telemetry("D1", alt=55.0)
        telem_d2_high = self._make_telemetry("D2", alt=55.0)

        # Same condition across two drones
        self.manager.check_telemetry(telem_d1_high, battery_fraction=0.90, current_time=10.0)
        self.manager.check_telemetry(telem_d2_high, battery_fraction=0.90, current_time=10.0)
        self.assertEqual(len(self.manager.get_all_incidents()), 2)

        # Second condition on D1 (low battery warning)
        self.manager.check_telemetry(telem_d1_high, battery_fraction=0.28, current_time=11.0)
        self.assertEqual(len(self.manager.get_all_incidents()), 3)

        inc_map = {(inc.drone_id, inc.condition_type): inc for inc in self.manager.get_all_incidents()}
        self.assertIn(("D1", "geofence_ceiling"), inc_map)
        self.assertIn(("D2", "geofence_ceiling"), inc_map)
        self.assertIn(("D1", "battery_failsafe"), inc_map)

        # Resolve D1 ceiling only
        telem_d1_ok_alt = self._make_telemetry("D1", alt=25.0)
        self.manager.check_telemetry(telem_d1_ok_alt, battery_fraction=0.28, current_time=12.0)
        active_map = {(inc.drone_id, inc.condition_type): inc for inc in self.manager.get_active_incidents()}
        self.assertNotIn(("D1", "geofence_ceiling"), active_map)
        self.assertIn(("D2", "geofence_ceiling"), active_map)
        self.assertIn(("D1", "battery_failsafe"), active_map)


if __name__ == "__main__":
    unittest.main()
