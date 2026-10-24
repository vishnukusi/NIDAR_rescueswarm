"""Guarded mission lifecycle state machine."""

from .models import MissionState


_ALLOWED_TRANSITIONS = {
    MissionState.IDLE: {MissionState.INITIALIZING},
    MissionState.INITIALIZING: {MissionState.SEARCHING, MissionState.ABORTED},
    MissionState.SEARCHING: {
        MissionState.RESCUE_IN_PROGRESS, MissionState.RETURNING, MissionState.ABORTED
    },
    MissionState.RESCUE_IN_PROGRESS: {
        MissionState.SEARCHING, MissionState.RETURNING, MissionState.ABORTED
    },
    MissionState.RETURNING: {MissionState.COMPLETED, MissionState.ABORTED},
    MissionState.COMPLETED: set(),
    MissionState.ABORTED: set(),
}


class InvalidMissionTransition(ValueError):
    pass


class MissionManager:
    def __init__(self) -> None:
        self.state = MissionState.IDLE
        self.history = [self.state]

    def transition_to(self, new_state: MissionState) -> None:
        if new_state not in _ALLOWED_TRANSITIONS[self.state]:
            raise InvalidMissionTransition(
                f"cannot transition mission from {self.state.value} to {new_state.value}"
            )
        self.state = new_state
        self.history.append(new_state)
