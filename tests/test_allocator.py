import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.allocator import AllocationError, assign_delivery_drone
from rescue_swarm_core.models import Drone, DroneRole, DroneStatus, RescueTask, TaskStatus


class AllocatorTests(unittest.TestCase):
    def setUp(self):
        self.drones = [
            Drone("S1", DroneRole.SEARCH, DroneStatus.READY, battery_fraction=0.99),
            Drone("D1", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.72),
            Drone("D2", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.91),
            Drone("D3", DroneRole.DELIVERY, DroneStatus.FAULT, battery_fraction=1.00),
        ]
        self.task = RescueTask("T1", 0.94, 0.5, 0.5, "S2", TaskStatus.GEOLOCATED)

    def test_selects_highest_battery_eligible_delivery_drone(self):
        result = assign_delivery_drone(self.task, self.drones)
        self.assertEqual(result, "D2")
        self.assertEqual(self.task.status, TaskStatus.ASSIGNED)
        self.assertEqual(self.task.assigned_drone_id, "D2")

    def test_search_drone_is_never_selected(self):
        self.drones[1].battery_fraction = 0.20
        self.drones[2].battery_fraction = 0.25
        with self.assertRaises(AllocationError):
            assign_delivery_drone(self.task, self.drones)

    def test_cannot_assign_same_task_twice(self):
        assign_delivery_drone(self.task, self.drones)
        with self.assertRaises(AllocationError):
            assign_delivery_drone(self.task, self.drones)


if __name__ == "__main__":
    unittest.main()
