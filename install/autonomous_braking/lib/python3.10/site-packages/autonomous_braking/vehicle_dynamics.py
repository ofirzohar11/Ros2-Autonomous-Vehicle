"""
═══════════════════════════════════════════════════════════════════════════════
    VEHICLE DYNAMICS MODULE
    Realistic vehicle physics simulation for AEB system
═══════════════════════════════════════════════════════════════════════════════
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple
from enum import Enum

from .config import PHYSICS, SAFETY, ROAD, LanePosition, VehicleStatus


class BrakeMode(Enum):
    """Braking intensity modes"""
    NONE = 0
    LIGHT = 1
    MODERATE = 2
    HARD = 3
    EMERGENCY = 4


@dataclass
class VehicleState:
    """Complete state of a vehicle"""
    # Position (meters)
    x: float = 0.0  # lateral position (across lanes)
    y: float = 0.0  # longitudinal position (along road)
    z: float = 0.0  # vertical (for visualization)
    
    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    
    # Acceleration (m/s²)
    ax: float = 0.0
    ay: float = 0.0
    
    # Orientation (radians)
    heading: float = 0.0
    steering_angle: float = 0.0
    
    # Lane
    current_lane: LanePosition = LanePosition.CENTER
    target_lane: Optional[LanePosition] = None
    lane_change_progress: float = 0.0
    
    # Status
    speed_kmh: float = 0.0
    is_braking: bool = False
    brake_mode: BrakeMode = BrakeMode.NONE
    status: VehicleStatus = VehicleStatus.SAFE
    
    def update_speed(self):
        """Update speed in km/h from velocity"""
        self.speed_kmh = math.sqrt(self.vx**2 + self.vy**2) * 3.6


@dataclass
class ObstacleState:
    """State of an obstacle"""
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    width: float = 1.0
    height: float = 1.0
    length: float = 1.0
    obstacle_type: str = "car"
    lane: LanePosition = LanePosition.CENTER
    is_moving: bool = True
    behavior: str = "steady"  # steady, braking, accelerating, crossing
    time_stationary: float = 0.0


class VehicleDynamics:
    """
    Advanced vehicle dynamics model with realistic physics
    """
    
    def __init__(self):
        self.state = VehicleState()
        self.physics = PHYSICS
        self.dt = 0.02  # 50Hz update rate
        
        # Lane geometry
        self.lane_centers = self._calculate_lane_centers()
        
    def _calculate_lane_centers(self) -> dict:
        """Calculate the center x-position of each lane"""
        total_road_width = ROAD.num_lanes * ROAD.lane_width
        centers = {}
        for lane in LanePosition:
            # Lane 0 (LEFT) is at negative x, Lane 2 (RIGHT) is at positive x
            offset = (lane.value - (ROAD.num_lanes - 1) / 2) * ROAD.lane_width
            centers[lane] = offset
        return centers
    
    def get_lane_center(self, lane: LanePosition) -> float:
        """Get the x-position of a lane's center"""
        return self.lane_centers.get(lane, 0.0)
    
    def update(self, throttle: float, brake: float, steering: float, dt: float = None):
        """
        Update vehicle state based on control inputs
        
        Args:
            throttle: 0.0 to 1.0
            brake: 0.0 to 1.0
            steering: -1.0 (left) to 1.0 (right)
            dt: time step in seconds
        """
        if dt is None:
            dt = self.dt
            
        # Clamp inputs
        throttle = max(0.0, min(1.0, throttle))
        brake = max(0.0, min(1.0, brake))
        steering = max(-1.0, min(1.0, steering))
        
        # Calculate longitudinal acceleration
        if brake > 0.01:
            # Braking
            self.state.is_braking = True
            if brake > 0.9:
                self.state.brake_mode = BrakeMode.EMERGENCY
                decel = self.physics.max_deceleration
            elif brake > 0.7:
                self.state.brake_mode = BrakeMode.HARD
                decel = self.physics.max_deceleration * 0.8
            elif brake > 0.4:
                self.state.brake_mode = BrakeMode.MODERATE
                decel = self.physics.normal_deceleration
            else:
                self.state.brake_mode = BrakeMode.LIGHT
                decel = self.physics.normal_deceleration * 0.5
            
            self.state.ay = -decel * brake
        else:
            # Acceleration
            self.state.is_braking = False
            self.state.brake_mode = BrakeMode.NONE
            
            # Acceleration decreases with speed (engine power curve)
            speed_factor = 1.0 - (self.state.speed_kmh / self.physics.max_speed_kmh) ** 2
            self.state.ay = self.physics.max_acceleration * throttle * max(0.1, speed_factor)
        
        # Update longitudinal velocity
        self.state.vy += self.state.ay * dt
        self.state.vy = max(0.0, min(self.state.vy, self.physics.max_speed_kmh / 3.6))
        
        # Handle lane changes
        if self.state.target_lane is not None:
            self._update_lane_change(dt)
        else:
            # Normal steering (minor adjustments)
            max_lateral_speed = 2.0  # m/s for minor adjustments
            self.state.vx = steering * max_lateral_speed * 0.3
        
        # Update position
        self.state.y += self.state.vy * dt
        self.state.x += self.state.vx * dt
        
        # Keep within road bounds
        max_x = (ROAD.num_lanes * ROAD.lane_width) / 2 + ROAD.shoulder_width
        self.state.x = max(-max_x, min(max_x, self.state.x))
        
        # Update speed display
        self.state.update_speed()
        
        # Update current lane based on position
        self._update_current_lane()
    
    def _update_lane_change(self, dt: float):
        """Handle smooth lane change transition"""
        if self.state.target_lane is None:
            return
            
        target_x = self.get_lane_center(self.state.target_lane)
        current_x = self.state.x
        dx = target_x - current_x
        
        # Smooth S-curve lane change
        self.state.lane_change_progress += dt / self.physics.lane_change_duration
        
        if self.state.lane_change_progress >= 1.0:
            # Lane change complete
            self.state.x = target_x
            self.state.vx = 0.0
            self.state.current_lane = self.state.target_lane
            self.state.target_lane = None
            self.state.lane_change_progress = 0.0
        else:
            # Smooth interpolation using sine curve
            t = self.state.lane_change_progress
            smooth_t = 0.5 - 0.5 * math.cos(t * math.pi)
            
            start_x = self.get_lane_center(self.state.current_lane)
            self.state.x = start_x + (target_x - start_x) * smooth_t
            self.state.vx = (target_x - start_x) * math.pi * math.sin(t * math.pi) / (2 * self.physics.lane_change_duration)
    
    def _update_current_lane(self):
        """Determine current lane based on x position"""
        if self.state.target_lane is not None:
            return  # Don't update during lane change
            
        min_dist = float('inf')
        closest_lane = LanePosition.CENTER
        
        for lane, center in self.lane_centers.items():
            dist = abs(self.state.x - center)
            if dist < min_dist:
                min_dist = dist
                closest_lane = lane
        
        self.state.current_lane = closest_lane
    
    def initiate_lane_change(self, direction: int) -> bool:
        """
        Start a lane change maneuver
        
        Args:
            direction: -1 for left, +1 for right
            
        Returns:
            True if lane change initiated, False if not possible
        """
        if self.state.target_lane is not None:
            return False  # Already changing lanes
            
        current = self.state.current_lane.value
        target = current + direction
        
        if target < 0 or target >= ROAD.num_lanes:
            return False  # Can't change to non-existent lane
        
        self.state.target_lane = LanePosition(target)
        self.state.lane_change_progress = 0.0
        return True
    
    def emergency_brake(self):
        """Apply maximum braking force"""
        self.state.is_braking = True
        self.state.brake_mode = BrakeMode.EMERGENCY
        self.state.ay = -self.physics.max_deceleration
    
    def calculate_stopping_distance(self) -> float:
        """Calculate minimum stopping distance at current speed"""
        v = self.state.vy  # m/s
        a = self.physics.max_deceleration
        reaction_dist = v * self.physics.reaction_time
        braking_dist = (v ** 2) / (2 * a)
        return reaction_dist + braking_dist
    
    def calculate_ttc(self, obstacle_distance: float, obstacle_velocity: float) -> float:
        """
        Calculate Time To Collision
        
        Args:
            obstacle_distance: distance to obstacle in meters
            obstacle_velocity: obstacle velocity (positive = same direction)
            
        Returns:
            Time to collision in seconds, or inf if no collision
        """
        relative_velocity = self.state.vy - obstacle_velocity
        
        if relative_velocity <= 0:
            return float('inf')  # Moving away or same speed
        
        return obstacle_distance / relative_velocity
    
    def determine_status(self, obstacle_distance: float, ttc: float) -> VehicleStatus:
        """Determine vehicle status based on safety metrics"""
        
        if obstacle_distance <= SAFETY.emergency_distance or ttc < 1.0:
            return VehicleStatus.EMERGENCY
        elif obstacle_distance <= SAFETY.danger_distance or ttc < 1.5:
            return VehicleStatus.DANGER
        elif obstacle_distance <= SAFETY.warning_distance or ttc < 2.5:
            return VehicleStatus.WARNING
        elif obstacle_distance <= SAFETY.caution_distance or ttc < SAFETY.time_to_collision_threshold:
            return VehicleStatus.CAUTION
        else:
            return VehicleStatus.SAFE


class TrafficVehicle:
    """
    Simulated traffic vehicle with simple AI behavior
    """
    
    def __init__(self, lane: LanePosition, initial_y: float, speed_kmh: float):
        self.state = ObstacleState()
        self.state.lane = lane
        self.state.y = initial_y
        self.state.vy = speed_kmh / 3.6
        self.state.x = self._get_lane_center(lane)
        self.state.obstacle_type = "car"
        self.state.width = 1.8
        self.state.length = 4.5
        self.state.height = 1.5
        
        # Behavior
        self.target_speed = speed_kmh / 3.6
        self.behavior_timer = 0.0
        
    def _get_lane_center(self, lane: LanePosition) -> float:
        """Get x position of lane center"""
        return (lane.value - (ROAD.num_lanes - 1) / 2) * ROAD.lane_width
    
    def update(self, dt: float, ego_y: float):
        """Update traffic vehicle position"""
        # Simple constant speed behavior
        self.state.y += self.state.vy * dt
        
        # Random speed variations
        self.behavior_timer += dt
        if self.behavior_timer > 3.0:
            self.behavior_timer = 0.0
            import random
            speed_change = random.uniform(-0.5, 0.5)
            self.state.vy = max(15/3.6, min(35/3.6, self.target_speed + speed_change))
        
        # Reset if too far ahead or behind
        if self.state.y - ego_y > 200 or ego_y - self.state.y > 50:
            import random
            self.state.y = ego_y + random.uniform(80, 150)
            self.state.lane = LanePosition(random.randint(0, ROAD.num_lanes - 1))
            self.state.x = self._get_lane_center(self.state.lane)
