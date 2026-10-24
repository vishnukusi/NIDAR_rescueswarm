import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.models import GeoPoint, Drone, DroneRole, RescueTask


class ModelTests(unittest.TestCase):
    def test_geopoint_rejects_invalid_latitude(self):
        with self.assertRaises(ValueError):
            GeoPoint(95.0, 12.0)

    def test_battery_fraction_must_be_valid(self):
        with self.assertRaises(ValueError):
            Drone("D1", DroneRole.DELIVERY, battery_fraction=1.5)

    def test_confidence_must_be_valid(self):
        with self.assertRaises(ValueError):
            RescueTask("T1", 1.5, 0.2, 0.3, "S1")


if __name__ == "__main__":
    unittest.main()
