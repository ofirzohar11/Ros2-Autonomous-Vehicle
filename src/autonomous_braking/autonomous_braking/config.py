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
class PIDConfig:
    """
    Gains for the longitudinal (speed) PID controller.

    The controller tracks a target speed by computing an acceleration
    command (m/s²) from the speed error (target − current):

        error      = target_speed − current_speed
        output     = Kp·error + Ki·∫error·dt + Kd·d(error)/dt

    A positive output means "throttle" (accelerate), a negative output
    means "ease off / light brake".  Emergency AEB braking always
    overrides the PID.
    """
    kp: float = 0.9       # proportional — reacts to the present error
    ki: float = 0.25      # integral     — removes steady-state offset
    kd: float = 0.05      # derivative   — damps overshoot
    # Output limits (m/s²): clamp the command between gentle braking and
    # the vehicle's maximum acceleration.
    output_min: float = -2.0
    output_max: float = 2.0
    # Anti-windup: cap the accumulated integral term so it can't build up
    # while the target is unreachable (e.g. during heavy braking).
    integral_limit: float = 8.0


@dataclass
class BrakePIDConfig:
    """
    Gains for the braking PID controller.

    Instead of fixed brake levels per danger-zone, the brake force is
    produced by a PID that tracks a safe following distance (a gap that
    grows with speed):

        desired_gap = min_gap + time_headway · current_speed
        error       = desired_gap − actual_distance   (positive = too close)
        brake       = clamp(Kp·error + Ki·∫error·dt + Kd·d(error)/dt, 0, 1)

    The closer we are inside the desired gap (and the faster we are
    closing it), the harder it brakes. The emergency range / critical
    time-to-collision still hard-override this with full braking.
    """
    kp: float = 0.08
    ki: float = 0.02
    kd: float = 0.02
    output_min: float = 0.0    # brake intensity never goes below 0
    output_max: float = 1.0    # ... or above full braking
    integral_limit: float = 40.0
    time_headway: float = 1.8  # seconds of gap to keep per unit of speed
    min_gap: float = 5.0       # metres of gap to keep at a standstill


@dataclass
class SafetyConfig:
    # FSM distance zones (metres). The EMERGENCY zone is everything closer
    # than danger_distance, so it needs no separate threshold.
    safe_distance: float = 30.0
    caution_distance: float = 20.0
    warning_distance: float = 15.0
    danger_distance: float = 10.0
    # Time-to-collision overrides (seconds). Only the danger/emergency
    # thresholds drive behaviour today.
    ttc_danger: float = 1.3
    ttc_emergency: float = 0.8


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
PID = PIDConfig()
BRAKE_PID = BrakePIDConfig()
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