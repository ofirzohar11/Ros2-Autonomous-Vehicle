"""
AUTONOMOUS EMERGENCY BRAKING SYSTEM - CONFIGURATION
ROS2 Humble | Python 3.10+
University Final Project - ADAS
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Tuple, Dict


class ObstacleType(Enum):
    CAR = "car"
    PEDESTRIAN = "pedestrian"
    CHILD = "child"
    BALL = "ball"
    FALLEN_TREE = "fallen_tree"
    MOTORCYCLE = "motorcycle"
    BICYCLE = "bicycle"
    ANIMAL = "animal"


class VehicleStatus(Enum):
    SAFE = "SAFE"
    CAUTION = "CAUTION"
    WARNING = "WARNING"
    DANGER = "DANGER"
    EMERGENCY = "EMERGENCY"
    BRAKING = "BRAKING"
    LANE_CHANGE = "LANE_CHANGE"
    STOPPED = "STOPPED"


class LanePosition(Enum):
    LEFT = 0
    CENTER = 1
    RIGHT = 2


class ObstacleBehavior(Enum):
    STATIC = "static"
    CROSSING = "crossing"
    SAME_DIRECTION = "same"
    OPPOSITE = "opposite"
    ERRATIC = "erratic"
    BRAKING = "braking"


@dataclass
class PhysicsConfig:
    max_speed_kmh: float = 120.0
    cruise_speed_kmh: float = 50.0
    max_acceleration: float = 2.0
    max_deceleration: float = 9.0
    normal_deceleration: float = 3.5
    comfort_deceleration: float = 2.0
    lane_change_duration: float = 2.5
    vehicle_length: float = 4.5
    vehicle_width: float = 1.8
    reaction_time: float = 0.1


@dataclass
class SafetyConfig:
    safe_distance: float = 30.0
    caution_distance: float = 20.0
    warning_distance: float = 15.0
    danger_distance: float = 10.0
    emergency_distance: float = 6.0
    ttc_safe: float = 4.0
    ttc_warning: float = 2.0
    ttc_danger: float = 1.3
    ttc_emergency: float = 0.8
    min_safe_gap: float = 3.0


@dataclass
class LidarConfig:
    max_range: float = 150.0
    min_range: float = 0.5
    angular_resolution: float = 1.0
    num_layers: int = 32
    # Forward-facing only: ±60° from front
    horizontal_fov: float = 120.0      # Total forward FOV in degrees
    noise_std: float = 0.02
    num_beams: int = 120               # One beam per degree across 120°


@dataclass
class RoadConfig:
    num_lanes: int = 3
    lane_width: float = 3.7
    road_length: float = 300.0
    shoulder_width: float = 2.5
    speed_limit_kmh: float = 100.0


@dataclass
class VisualizationConfig:
    width: int = 1400
    height: int = 800
    fps: int = 60
    horizon_y: int = 280
    road_vanishing_width: int = 60
    road_near_width: int = 700


@dataclass
class TrafficConfig:
    min_traffic_vehicles: int = 3
    max_traffic_vehicles: int = 6
    traffic_speed_variance: float = 0.30
    # Spawn every 3-5 seconds → event every ~5 sec
    obstacle_spawn_interval: Tuple[float, float] = (1.0, 3.0)
    obstacle_spawn_probability: float = 0.90
    # Spawn closer so ego reaches them quickly
    min_spawn_distance: float = 20.0
    max_spawn_distance: float = 50.0
    min_vehicle_spacing: float = 10.0
    obstacle_weights: Dict = None

    def __post_init__(self):
        if self.obstacle_weights is None:
            self.obstacle_weights = {
                ObstacleType.CAR: 0.30,
                ObstacleType.PEDESTRIAN: 0.15,
                ObstacleType.CHILD: 0.15,
                ObstacleType.BALL: 0.08,
                ObstacleType.FALLEN_TREE: 0.10,
                ObstacleType.MOTORCYCLE: 0.10,
                ObstacleType.BICYCLE: 0.07,
                ObstacleType.ANIMAL: 0.05,
            }


# Global instances
PHYSICS = PhysicsConfig()
SAFETY = SafetyConfig()
LIDAR_CFG = LidarConfig()
ROAD = RoadConfig()
VISUALIZATION = VisualizationConfig()
TRAFFIC = TrafficConfig()


class Topics:
    LIDAR_SCAN = '/aeb/lidar/scan'
    OBSTACLE_NEAREST = '/aeb/obstacle/nearest'
    OBSTACLES_LIST = '/aeb/obstacles'
    EGO_SPEED = '/aeb/ego/speed'
    EGO_LANE = '/aeb/ego/lane'
    EGO_POSITION = '/aeb/ego/position'
    EGO_STATUS = '/aeb/ego/status'
    BRAKE_COMMAND = '/aeb/control/brake'
    LANE_CHANGE = '/aeb/control/lane_change'
    FULL_STATE = '/aeb/full_state'
    STATISTICS = '/aeb/statistics'