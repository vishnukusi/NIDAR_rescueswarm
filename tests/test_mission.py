import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.mission import InvalidMissionTransition, MissionManager
from rescue_swarm_core.models import MissionState


class MissionManagerTests(unittest.TestCase):
    def test_valid_startup_sequence(self):
        mission = MissionManager()
        mission.transition_to(MissionState.INITIALIZING)
        mission.transition_to(MissionState.SEARCHING)
        mission.transition_to(MissionState.RESCUE_IN_PROGRESS)
        self.assertEqual(mission.state, MissionState.RESCUE_IN_PROGRESS)

    def test_rejects_skipping_initialization(self):
        mission = MissionManager()
        with self.assertRaises(InvalidMissionTransition):
            mission.transition_to(MissionState.SEARCHING)

    def test_completed_mission_is_terminal(self):
        mission = MissionManager()
        mission.transition_to(MissionState.INITIALIZING)
        mission.transition_to(MissionState.SEARCHING)
        mission.transition_to(MissionState.RETURNING)
        mission.transition_to(MissionState.COMPLETED)
        with self.assertRaises(InvalidMissionTransition):
            mission.transition_to(MissionState.INITIALIZING)


if __name__ == "__main__":
    unittest.main()
