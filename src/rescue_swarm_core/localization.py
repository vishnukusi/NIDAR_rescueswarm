"""Camera geometry, ray-casting, and survivor localization for RescueSwarm.

Estimates ground survivor coordinates from camera detections, camera parameters,
and drone telemetry using 3D ray-plane intersection. Never assigns drone GPS
directly to survivor location.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from .models import GeoPoint
from .perception import BoundingBox, Detection


class LocalizationError(Exception):
    """Raised when target localization fails (e.g., ray points above horizon)."""


@dataclass(frozen=True)
class CameraCalibration:
    """Camera sensor and optical mounting parameters.

    mount_pitch_deg: Gimbal pitch relative to vehicle horizontal:
                     -90.0 = nadir (pointing straight down).
                       0.0 = forward-facing horizontal.
    """
    image_width: int = 1920
    image_height: int = 1080
    hfov_deg: float = 84.0
    vfov_deg: Optional[float] = None
    mount_pitch_deg: float = -90.0
    mount_roll_deg: float = 0.0
    mount_yaw_deg: float = 0.0

    def __post_init__(self) -> None:
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image dimensions must be positive integers")
        if not (0.0 < self.hfov_deg < 180.0):
            raise ValueError("hfov_deg must be between 0 and 180 degrees")
        if self.vfov_deg is not None and not (0.0 < self.vfov_deg < 180.0):
            raise ValueError("vfov_deg must be between 0 and 180 degrees")
        if not all(math.isfinite(v) for v in (self.mount_pitch_deg, self.mount_roll_deg, self.mount_yaw_deg)):
            raise ValueError("mount angles must be finite numbers")

    @property
    def effective_vfov_deg(self) -> float:
        if self.vfov_deg is not None:
            return self.vfov_deg
        # Compute vertical FOV from aspect ratio: tan(vfov/2) = tan(hfov/2) * (H / W)
        half_hfov_rad = math.radians(self.hfov_deg / 2.0)
        half_vfov_rad = math.atan(math.tan(half_hfov_rad) * (self.image_height / self.image_width))
        return math.degrees(half_vfov_rad * 2.0)


@dataclass(frozen=True)
class DroneTelemetry:
    """Telemetry of the observing drone at the moment of image capture."""
    drone_id: str
    position: GeoPoint
    altitude_agl_m: float
    heading_deg: float  # Yaw: 0 = North, 90 = East, 180 = South, 270 = West
    pitch_deg: float = 0.0  # Vehicle pitch: + nose up
    roll_deg: float = 0.0   # Vehicle roll: + right wing down
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        if not self.drone_id:
            raise ValueError("drone_id cannot be empty")
        if not math.isfinite(self.altitude_agl_m) or self.altitude_agl_m <= 0.0:
            raise ValueError("altitude_agl_m must be strictly positive and finite for ground localization")
        if not all(math.isfinite(v) for v in (self.heading_deg, self.pitch_deg, self.roll_deg)):
            raise ValueError("telemetry angles must be finite numbers")


@dataclass(frozen=True)
class GeolocationEstimate:
    """Estimated ground location of a detected target with uncertainty.

    Note: uncertainty_radius_m is an engineering heuristic scaling with altitude
    and incidence angle, not a statistically calibrated error covariance.
    """
    location: GeoPoint
    offset_north_m: float
    offset_east_m: float
    ground_distance_m: float
    slant_range_m: float
    uncertainty_radius_m: float

    def __post_init__(self) -> None:
        if self.location is None or not isinstance(self.location, GeoPoint):
            raise ValueError("location must be a valid GeoPoint instance")
        if not math.isfinite(self.uncertainty_radius_m) or self.uncertainty_radius_m < 0.0:
            raise ValueError("uncertainty_radius_m must be a non-negative finite number")
        if not all(
            math.isfinite(v)
            for v in (self.offset_north_m, self.offset_east_m, self.ground_distance_m, self.slant_range_m)
        ):
            raise ValueError("geometric offsets and ranges must be finite numbers")


def haversine_distance_m(p1: GeoPoint, p2: GeoPoint) -> float:
    """Calculate the great-circle distance between two GeoPoints in meters."""
    r_earth = 6371000.0  # Earth mean radius in meters
    phi1 = math.radians(p1.latitude)
    phi2 = math.radians(p2.latitude)
    delta_phi = math.radians(p2.latitude - p1.latitude)
    delta_lambda = math.radians(p2.longitude - p1.longitude)

    a = (math.sin(delta_phi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r_earth * c


def estimate_survivor_location(
    detection: Detection,
    telemetry: DroneTelemetry,
    camera: CameraCalibration,
    ground_elevation_m: float = 0.0,
) -> GeolocationEstimate:
    """Project a 2D bounding box detection onto the ground plane using 3D ray-casting.

    1. Computes ray direction in camera optical frame from normalized (cx, cy).
    2. Rotates ray into vehicle body frame accounting for gimbal orientation.
    3. Rotates ray into World NED (North-East-Down) frame using vehicle attitude.
    4. Calculates intersection with horizontal terrain plane at AGL altitude.
    5. Translates relative (North, East) offsets to WGS-84 GeoPoint coordinates.
    6. Estimates localization uncertainty based on geometry and detection size.
    """
    # Validate input types
    if not isinstance(detection, Detection) or not isinstance(detection.bbox, BoundingBox):
        raise ValueError("detection must be a valid Detection instance with a BoundingBox")
    if not isinstance(telemetry, DroneTelemetry):
        raise ValueError("telemetry must be a valid DroneTelemetry instance")
    if not isinstance(camera, CameraCalibration):
        raise ValueError("camera must be a valid CameraCalibration instance")

    # 1. Camera optical frame ray vector
    # Normalized image coordinates: cx, cy in [0, 1]
    cx = detection.bbox.center_x
    cy = detection.bbox.center_y

    hfov_rad = math.radians(camera.hfov_deg)
    vfov_rad = math.radians(camera.effective_vfov_deg)

    # Angles from camera optical axis (+Zc forward, +Xc right, +Yc down)
    tan_x = (cx - 0.5) * 2.0 * math.tan(hfov_rad / 2.0)
    tan_y = (cy - 0.5) * 2.0 * math.tan(vfov_rad / 2.0)

    # Unnormalized ray in camera frame
    ray_cam = [tan_x, tan_y, 1.0]
    cam_norm = math.sqrt(ray_cam[0]**2 + ray_cam[1]**2 + ray_cam[2]**2)
    ray_cam = [v / cam_norm for v in ray_cam]

    # 2. Camera to Vehicle Body transform
    # Base alignment (forward-facing camera at 0 pitch, 0 roll, 0 yaw):
    #   Camera optical Z (+forward) -> Vehicle Body +X (forward)
    #   Camera optical X (+right)   -> Vehicle Body +Y (right)
    #   Camera optical Y (+down)    -> Vehicle Body +Z (down)
    r_cam_base = [ray_cam[2], ray_cam[0], ray_cam[1]]

    # Gimbal mounting angles relative to vehicle body:
    #   mount_pitch_deg: -90 deg = nadir (pointing down) -> tilt = +90 deg around Body Y
    #   mount_roll_deg:  roll offset around Body X
    #   mount_yaw_deg:   yaw offset around Body Z
    tilt_m = math.radians(-camera.mount_pitch_deg)
    roll_m = math.radians(camera.mount_roll_deg)
    yaw_m = math.radians(camera.mount_yaw_deg)

    # 1. Roll rotation around Body X
    cr_m, sr_m = math.cos(roll_m), math.sin(roll_m)
    rx1 = r_cam_base[0]
    ry1 = r_cam_base[1] * cr_m - r_cam_base[2] * sr_m
    rz1 = r_cam_base[1] * sr_m + r_cam_base[2] * cr_m

    # 2. Pitch rotation around Body Y
    cp_m, sp_m = math.cos(tilt_m), math.sin(tilt_m)
    rx2 = rx1 * cp_m - rz1 * sp_m
    ry2 = ry1
    rz2 = rx1 * sp_m + rz1 * cp_m

    # 3. Yaw rotation around Body Z
    cy_m, sy_m = math.cos(yaw_m), math.sin(yaw_m)
    r_body = [
        rx2 * cy_m - ry2 * sy_m,
        rx2 * sy_m + ry2 * cy_m,
        rz2,
    ]

    # 3. Vehicle Body to World NED rotation
    # Drone attitude: heading (yaw psi), pitch theta, roll phi
    psi = math.radians(telemetry.heading_deg)
    theta = math.radians(telemetry.pitch_deg)
    phi = math.radians(telemetry.roll_deg)

    # Standard NED rotation matrix R_bn (ZYX Euler angles):
    # R = R_z(psi) * R_y(theta) * R_x(phi)
    c_psi, s_psi = math.cos(psi), math.sin(psi)
    c_th, s_th = math.cos(theta), math.sin(theta)
    c_ph, s_ph = math.cos(phi), math.sin(phi)

    # Compute ray in NED frame:
    r_ned_x = (c_psi * c_th) * r_body[0] + (c_psi * s_th * s_ph - s_psi * c_ph) * r_body[1] + (c_psi * s_th * c_ph + s_psi * s_ph) * r_body[2]
    r_ned_y = (s_psi * c_th) * r_body[0] + (s_psi * s_th * s_ph + c_psi * c_ph) * r_body[1] + (s_psi * s_th * c_ph - c_psi * s_ph) * r_body[2]
    r_ned_z = (-s_th) * r_body[0] + (c_th * s_ph) * r_body[1] + (c_th * c_ph) * r_body[2]

    # Normalize NED ray
    norm_ned = math.sqrt(r_ned_x**2 + r_ned_y**2 + r_ned_z**2)
    r_ned_x /= norm_ned
    r_ned_y /= norm_ned
    r_ned_z /= norm_ned

    # 4. Ray-Plane intersection
    # Drone altitude AGL h > 0. Ground is at +h down along Z_ned.
    # Intersection requires r_ned_z > 0 (pointing downward).
    if r_ned_z <= 1e-4:
        raise LocalizationError(
            f"Camera ray points at or above horizon (Down component = {r_ned_z:.4f}). "
            "Cannot intersect ground plane."
        )

    scale = telemetry.altitude_agl_m / r_ned_z
    offset_north = scale * r_ned_x
    offset_east = scale * r_ned_y
    ground_dist = math.sqrt(offset_north**2 + offset_east**2)
    slant_range = scale

    # 5. Convert NED offsets to WGS-84 coordinates
    meters_per_deg_lat = 111132.954
    meters_per_deg_lon = 111132.954 * math.cos(math.radians(telemetry.position.latitude))

    if abs(meters_per_deg_lon) < 1.0:
        meters_per_deg_lon = 1.0

    target_lat = telemetry.position.latitude + (offset_north / meters_per_deg_lat)
    target_lon = telemetry.position.longitude + (offset_east / meters_per_deg_lon)

    # Clamp coordinates to valid range
    target_lat = max(-90.0, min(90.0, target_lat))
    target_lon = max(-180.0, min(180.0, target_lon))

    # 6. Localization Uncertainty Estimation
    # Uncertainty scales with altitude, incidence angle (1 / r_ned_z^2), and bounding box angular size
    bbox_diag = math.sqrt(detection.bbox.width**2 + detection.bbox.height**2)
    angular_spread = bbox_diag * math.radians(camera.hfov_deg)
    geometric_uncertainty = (telemetry.altitude_agl_m * angular_spread) / max(0.1, r_ned_z**2)
    baseline_gps_uncertainty = 1.5  # Typical simulated GPS standard deviation in meters
    total_uncertainty = math.sqrt(baseline_gps_uncertainty**2 + geometric_uncertainty**2)

    return GeolocationEstimate(
        location=GeoPoint(latitude=target_lat, longitude=target_lon, altitude_m=ground_elevation_m),
        offset_north_m=offset_north,
        offset_east_m=offset_east,
        ground_distance_m=ground_dist,
        slant_range_m=slant_range,
        uncertainty_radius_m=total_uncertainty,
    )
