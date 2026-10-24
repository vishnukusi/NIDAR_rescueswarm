"""Search coverage path planning for NIDAR RescueSwarm.

Partitions operational search zones into non-overlapping subsectors for search drones (S1, S2, S3)
and generates boustrophedon (lawnmower) flight paths based on camera FoV, flight altitude, and
sidelap requirements.
"""

from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

from .localization import CameraCalibration, haversine_distance_m
from .models import GeoPoint

METERS_PER_DEG_LAT = 111132.954


@dataclass(frozen=True)
class SearchArea:
    """Rectangular geographic search boundary."""
    min_lat: float
    min_lon: float
    max_lat: float
    max_lon: float

    def __post_init__(self) -> None:
        if self.min_lat >= self.max_lat:
            raise ValueError(f"min_lat ({self.min_lat}) must be strictly less than max_lat ({self.max_lat})")
        if self.min_lon >= self.max_lon:
            raise ValueError(f"min_lon ({self.min_lon}) must be strictly less than max_lon ({self.max_lon})")
        if not (-90.0 <= self.min_lat <= 90.0 and -90.0 <= self.max_lat <= 90.0):
            raise ValueError("Latitudes must be in [-90, 90]")
        if not (-180.0 <= self.min_lon <= 180.0 and -180.0 <= self.max_lon <= 180.0):
            raise ValueError("Longitudes must be in [-180, 180]")

    @property
    def center(self) -> GeoPoint:
        return GeoPoint(
            latitude=(self.min_lat + self.max_lat) / 2.0,
            longitude=(self.min_lon + self.max_lon) / 2.0,
        )

    @property
    def height_m(self) -> float:
        """North-South extent in meters."""
        return (self.max_lat - self.min_lat) * METERS_PER_DEG_LAT

    @property
    def width_m(self) -> float:
        """East-West extent in meters."""
        mid_lat = (self.min_lat + self.max_lat) / 2.0
        meters_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(mid_lat))
        return (self.max_lon - self.min_lon) * meters_per_deg_lon

    @property
    def area_m2(self) -> float:
        """Total area in square meters."""
        return self.height_m * self.width_m

    def contains(self, point: GeoPoint) -> bool:
        """Check if a GeoPoint lies within this search area."""
        return (self.min_lat <= point.latitude <= self.max_lat and
                self.min_lon <= point.longitude <= self.max_lon)


@dataclass
class SearchSector:
    """Assigned search subzone and planned flight path for an individual search drone."""
    drone_id: str
    area: SearchArea
    waypoints: List[GeoPoint]
    total_distance_m: float
    estimated_duration_s: float
    swath_width_m: float
    track_spacing_m: float


@dataclass
class CoveragePlan:
    """Full swarm search plan partitioning the operational area among all search drones."""
    operational_area: SearchArea
    sectors: Dict[str, SearchSector]
    altitude_agl_m: float
    swath_width_m: float
    track_spacing_m: float
    total_distance_m: float
    max_duration_s: float


def compute_ground_swath_width(camera: CameraCalibration, altitude_agl_m: float) -> float:
    """Calculate camera ground footprint width in meters perpendicular to flight heading."""
    if altitude_agl_m <= 0.0:
        raise ValueError("altitude_agl_m must be strictly positive")
    hfov_rad = math.radians(camera.hfov_deg)
    return 2.0 * altitude_agl_m * math.tan(hfov_rad / 2.0)


def plan_coverage(
    search_area: SearchArea,
    search_drone_ids: List[str],
    camera: CameraCalibration,
    altitude_agl_m: float,
    overlap_fraction: float = 0.20,
    cruise_speed_mps: float = 8.0,
    sweep_direction: str = "north_south",
) -> CoveragePlan:
    """Generate optimal, non-overlapping search coverage paths for the search fleet.

    Args:
        search_area: The overall rectangular bounding area to be searched.
        search_drone_ids: List of search drone IDs (e.g. ["S1", "S2", "S3"]).
        camera: Camera optical calibration parameters.
        altitude_agl_m: Patrol altitude above ground level.
        overlap_fraction: Lateral track overlap (e.g., 0.20 for 20% sidelap).
        cruise_speed_mps: Nominal flight velocity in meters per second.
        sweep_direction: 'north_south' or 'east_west'.

    Returns:
        CoveragePlan containing partitioned sectors and waypoints for each drone.
    """
    if not search_drone_ids:
        raise ValueError("search_drone_ids list cannot be empty")
    if not (0.0 <= overlap_fraction < 1.0):
        raise ValueError("overlap_fraction must be in [0.0, 1.0)")
    if cruise_speed_mps <= 0.0:
        raise ValueError("cruise_speed_mps must be strictly positive")

    # 1. Compute ground footprint and spacing
    swath_width = compute_ground_swath_width(camera, altitude_agl_m)
    track_spacing = swath_width * (1.0 - overlap_fraction)
    if track_spacing <= 0:
        track_spacing = swath_width

    # 2. Partition operational area among search drones along longitude (East-West partition)
    num_drones = len(search_drone_ids)
    delta_lon_total = search_area.max_lon - search_area.min_lon
    delta_lon_per_drone = delta_lon_total / num_drones

    sectors: Dict[str, SearchSector] = {}
    total_fleet_dist = 0.0
    max_fleet_duration = 0.0

    mid_lat = (search_area.min_lat + search_area.max_lat) / 2.0
    meters_per_deg_lon = METERS_PER_DEG_LAT * math.cos(math.radians(mid_lat))
    track_spacing_deg_lon = track_spacing / meters_per_deg_lon

    for idx, drone_id in enumerate(search_drone_ids):
        sec_min_lon = search_area.min_lon + idx * delta_lon_per_drone
        sec_max_lon = sec_min_lon + delta_lon_per_drone
        sub_area = SearchArea(
            min_lat=search_area.min_lat,
            min_lon=sec_min_lon,
            max_lat=search_area.max_lat,
            max_lon=sec_max_lon,
        )

        # 3. Generate boustrophedon (lawnmower) tracks across this subsector
        waypoints: List[GeoPoint] = []
        sec_width_m = sub_area.width_m
        num_tracks = max(1, math.ceil(sec_width_m / track_spacing))
        actual_lon_step = (sec_max_lon - sec_min_lon) / (num_tracks + 1)

        # Sweep lanes back and forth
        sweep_north = True
        for track_i in range(num_tracks):
            lon = sec_min_lon + (track_i + 0.5) * (sec_max_lon - sec_min_lon) / num_tracks
            if sweep_north:
                waypoints.append(GeoPoint(latitude=sub_area.min_lat, longitude=lon, altitude_m=altitude_agl_m))
                waypoints.append(GeoPoint(latitude=sub_area.max_lat, longitude=lon, altitude_m=altitude_agl_m))
            else:
                waypoints.append(GeoPoint(latitude=sub_area.max_lat, longitude=lon, altitude_m=altitude_agl_m))
                waypoints.append(GeoPoint(latitude=sub_area.min_lat, longitude=lon, altitude_m=altitude_agl_m))
            sweep_north = not sweep_north

        # 4. Calculate total sector flight distance and duration
        sec_dist = 0.0
        for i in range(len(waypoints) - 1):
            sec_dist += haversine_distance_m(waypoints[i], waypoints[i + 1])

        sec_duration = sec_dist / cruise_speed_mps
        total_fleet_dist += sec_dist
        max_fleet_duration = max(max_fleet_duration, sec_duration)

        sectors[drone_id] = SearchSector(
            drone_id=drone_id,
            area=sub_area,
            waypoints=waypoints,
            total_distance_m=sec_dist,
            estimated_duration_s=sec_duration,
            swath_width_m=swath_width,
            track_spacing_m=track_spacing,
        )

    return CoveragePlan(
        operational_area=search_area,
        sectors=sectors,
        altitude_agl_m=altitude_agl_m,
        swath_width_m=swath_width,
        track_spacing_m=track_spacing,
        total_distance_m=total_fleet_dist,
        max_duration_s=max_fleet_duration,
    )
