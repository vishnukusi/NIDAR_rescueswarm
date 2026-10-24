"""Portable mission logic for NIDAR RescueSwarm."""

from .allocator import AllocationError, assign_delivery_drone
from .association import (
    AssociationDecision,
    AssociationStatus,
    SurvivorAssociator,
)
from .approach_controller import (
    ApproachControllerConfig,
    ApproachState,
    SearchDroneApproachController,
    TargetTrackingEstimate,
    VisualServoCommand,
)
from .coverage_planner import (
    CoveragePlan,
    SearchArea,
    SearchSector,
    compute_ground_swath_width,
    plan_coverage,
)
from .delivery import (
    DeliveryMissionController,
    DeliveryProfile,
    DeliveryState,
    DeliveryVerificationMethod,
)
from .flight_bridge import (
    AutopilotType,
    BridgeTelemetryState,
    FlightBridgeConfig,
    MAVLinkFlightBridge,
    VehicleConnectionState,
)
from .fleet_flight_bridge import DEFAULT_FLEET_PORT_MAP, FleetFlightBridgeManager
from .mavlink_emulator import MockSITLEmulator
from .localization import (
    CameraCalibration,
    DroneTelemetry,
    GeolocationEstimate,
    LocalizationError,
    estimate_survivor_location,
    haversine_distance_m,
)
from .mission import InvalidMissionTransition, MissionManager
from .models import (
    Drone,
    DroneRole,
    DroneStatus,
    GeoPoint,
    MissionState,
    RescueTask,
    TaskStatus,
)
from .perception import (
    BoundingBox,
    Detection,
    PerceptionEngine,
    YOLOPerceptionAdapter,
    annotate_detection_image,
    extract_frame_from_video,
    inspect_model_weights,
)
from .safety_manager import (
    BatteryFailsafeConfig,
    GeofenceConfig,
    SafetyAction,
    SafetyAlert,
    SafetyIncident,
    SafetyLevel,
    SafetyManager,
)
from .swarm_manager import SurvivorRecord, SwarmManager
from .mission_sim import (
    CommsLossEvent,
    DeliveryOutcomeSpec,
    DroneFaultEvent,
    LowBatteryEvent,
    MissionScenarioConfig,
    MissionSimulationReport,
    ObservationEvent,
    SimulatedMissionRunner,
    create_scenario,
    validate_scenario_config,
)

__all__ = [
    "AllocationError",
    "assign_delivery_drone",
    "AssociationDecision",
    "AssociationStatus",
    "SurvivorAssociator",
    "ApproachControllerConfig",
    "ApproachState",
    "SearchDroneApproachController",
    "TargetTrackingEstimate",
    "VisualServoCommand",
    "CoveragePlan",
    "SearchArea",
    "SearchSector",
    "compute_ground_swath_width",
    "plan_coverage",
    "DeliveryMissionController",
    "DeliveryProfile",
    "DeliveryState",
    "DeliveryVerificationMethod",
    "CameraCalibration",
    "DroneTelemetry",
    "GeolocationEstimate",
    "LocalizationError",
    "estimate_survivor_location",
    "haversine_distance_m",
    "InvalidMissionTransition",
    "MissionManager",
    "Drone",
    "DroneRole",
    "DroneStatus",
    "GeoPoint",
    "MissionState",
    "RescueTask",
    "TaskStatus",
    "BoundingBox",
    "Detection",
    "PerceptionEngine",
    "YOLOPerceptionAdapter",
    "extract_frame_from_video",
    "BatteryFailsafeConfig",
    "GeofenceConfig",
    "SafetyAction",
    "SafetyAlert",
    "SafetyIncident",
    "SafetyLevel",
    "SafetyManager",
    "SurvivorRecord",
    "SwarmManager",
    "CommsLossEvent",
    "DeliveryOutcomeSpec",
    "DroneFaultEvent",
    "LowBatteryEvent",
    "MissionScenarioConfig",
    "MissionSimulationReport",
    "ObservationEvent",
    "SimulatedMissionRunner",
    "create_scenario",
    "validate_scenario_config",
]
