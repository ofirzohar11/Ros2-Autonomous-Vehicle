"""
═══════════════════════════════════════════════════════════════════════════════
    OBSTACLE MANAGER MODULE
    Handles spawning, behavior, and management of various obstacles
    
    Supported obstacles:
    - Vehicles (cars, motorcycles, bicycles)
    - Pedestrians (adults, children)
    - Objects (ball, fallen tree, debris)
    - Animals
═══════════════════════════════════════════════════════════════════════════════
"""

import random
import math
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Tuple
from enum import Enum

from .config import (
    ObstacleType, LanePosition, ROAD, TRAFFIC, SAFETY,
    PHYSICS
)


class ObstacleBehavior(Enum):
    """Behavior patterns for obstacles"""
    STATIC = "static"           # Not moving
    CROSSING = "crossing"       # Moving across the road
    SAME_DIRECTION = "same"     # Moving in same direction
    OPPOSITE = "opposite"       # Coming from opposite direction
    ERRATIC = "erratic"         # Unpredictable movement
    BRAKING = "braking"         # Slowing down
    ACCELERATING = "accelerating"


@dataclass
class Obstacle:
    """Complete obstacle representation"""
    id: int
    obstacle_type: ObstacleType
    
    # Position (meters)
    x: float = 0.0          # Lateral position
    y: float = 0.0          # Longitudinal position
    
    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    
    # Dimensions (meters)
    width: float = 1.0
    length: float = 1.0
    height: float = 1.0
    
    # State
    lane: LanePosition = LanePosition.CENTER
    behavior: ObstacleBehavior = ObstacleBehavior.STATIC
    is_active: bool = True
    time_alive: float = 0.0
    time_stationary: float = 0.0
    
    # Visual properties
    color: Tuple[int, int, int] = (255, 0, 0)
    
    # Behavior parameters
    target_x: Optional[float] = None
    crossing_speed: float = 0.0
    
    def get_distance_to(self, ego_y: float) -> float:
        """Calculate distance from ego vehicle"""
        return self.y - ego_y
    
    def get_bounding_box(self) -> Tuple[float, float, float, float]:
        """Get bounding box (x_min, x_max, y_min, y_max)"""
        return (
            self.x - self.width / 2,
            self.x + self.width / 2,
            self.y - self.length / 2,
            self.y + self.length / 2
        )


class ObstacleManager:
    """
    Manages obstacle spawning, behavior, and lifecycle
    """
    
    def __init__(self):
        self.obstacles: List[Obstacle] = []
        self.next_id = 0
        self.spawn_timer = 0.0
        self.spawn_interval = random.uniform(*TRAFFIC.obstacle_spawn_interval)
        
        # Obstacle templates
        self.templates = self._create_templates()
        
    def _create_templates(self) -> Dict[ObstacleType, dict]:
        """Create obstacle templates with properties"""
        return {
            ObstacleType.CAR: {
                'width': 1.8,
                'length': 4.5,
                'height': 1.5,
                'color': (180, 50, 50),
                'behaviors': [
                    ObstacleBehavior.SAME_DIRECTION,
                    ObstacleBehavior.BRAKING,
                    ObstacleBehavior.ERRATIC
                ],
                'speed_range': (60, 100),  # km/h
            },
            ObstacleType.PEDESTRIAN: {
                'width': 0.5,
                'length': 0.5,
                'height': 1.7,
                'color': (255, 165, 0),
                'behaviors': [ObstacleBehavior.CROSSING],
                'speed_range': (4, 7),  # km/h
            },
            ObstacleType.CHILD: {
                'width': 0.4,
                'length': 0.4,
                'height': 1.2,
                'color': (255, 200, 100),
                'behaviors': [ObstacleBehavior.CROSSING, ObstacleBehavior.ERRATIC],
                'speed_range': (6, 12),  # km/h - running child
            },
            ObstacleType.BALL: {
                'width': 0.3,
                'length': 0.3,
                'height': 0.3,
                'color': (255, 100, 100),
                'behaviors': [ObstacleBehavior.CROSSING],
                'speed_range': (10, 20),  # km/h - rolling ball
            },
            ObstacleType.FALLEN_TREE: {
                'width': 4.0,
                'length': 0.8,
                'height': 0.8,
                'color': (101, 67, 33),
                'behaviors': [ObstacleBehavior.STATIC],
                'speed_range': (0, 0),
            },
            ObstacleType.MOTORCYCLE: {
                'width': 0.8,
                'length': 2.2,
                'height': 1.4,
                'color': (100, 100, 200),
                'behaviors': [
                    ObstacleBehavior.SAME_DIRECTION,
                    ObstacleBehavior.ERRATIC
                ],
                'speed_range': (70, 120),
            },
            ObstacleType.BICYCLE: {
                'width': 0.6,
                'length': 1.8,
                'height': 1.5,
                'color': (50, 200, 100),
                'behaviors': [
                    ObstacleBehavior.SAME_DIRECTION,
                    ObstacleBehavior.CROSSING
                ],
                'speed_range': (15, 30),
            },
            ObstacleType.ANIMAL: {
                'width': 0.4,
                'length': 0.8,
                'height': 0.5,
                'color': (139, 90, 43),
                'behaviors': [ObstacleBehavior.CROSSING, ObstacleBehavior.ERRATIC],
                'speed_range': (10, 40),
            },
        }
    
    def spawn_obstacle(self, ego_y: float, ego_lane: LanePosition,
                      force_type: Optional[ObstacleType] = None) -> Optional[Obstacle]:
        """
        Spawn a new obstacle
        
        Args:
            ego_y: Ego vehicle longitudinal position
            ego_lane: Ego vehicle current lane
            force_type: Force a specific obstacle type
        """
        # Choose obstacle type based on weights
        if force_type:
            obs_type = force_type
        else:
            types = list(TRAFFIC.obstacle_weights.keys())
            weights = list(TRAFFIC.obstacle_weights.values())
            obs_type = random.choices(types, weights=weights)[0]
        
        template = self.templates[obs_type]
        
        # Choose behavior
        behavior = random.choice(template['behaviors'])
        
        # Determine spawn position based on behavior
        spawn_y = ego_y + random.uniform(80, 150)  # Ahead of ego
        
        if behavior == ObstacleBehavior.CROSSING:
            # Spawn at side of road
            side = random.choice([-1, 1])
            spawn_x = side * (ROAD.num_lanes * ROAD.lane_width / 2 + ROAD.shoulder_width)
            target_x = -spawn_x  # Cross to other side
            speed = random.uniform(*template['speed_range']) / 3.6
            vx = -side * speed
            vy = 0.0
            lane = ego_lane  # Will cross ego's lane
        elif behavior == ObstacleBehavior.SAME_DIRECTION:
            # In a lane, moving same direction
            lane = random.choice(list(LanePosition))
            spawn_x = self._get_lane_center(lane)
            target_x = None
            vx = 0.0
            vy = random.uniform(*template['speed_range']) / 3.6
        elif behavior == ObstacleBehavior.BRAKING:
            # Vehicle braking ahead in ego's lane
            lane = ego_lane
            spawn_x = self._get_lane_center(lane)
            target_x = None
            vx = 0.0
            vy = random.uniform(50, 80) / 3.6  # Will slow down
        elif behavior == ObstacleBehavior.STATIC:
            # Static obstacle in lane
            lane = random.choice([ego_lane, random.choice(list(LanePosition))])
            spawn_x = self._get_lane_center(lane)
            target_x = None
            vx = 0.0
            vy = 0.0
        elif behavior == ObstacleBehavior.ERRATIC:
            # Unpredictable movement
            lane = random.choice(list(LanePosition))
            spawn_x = self._get_lane_center(lane)
            target_x = None
            speed = random.uniform(*template['speed_range']) / 3.6
            vx = random.uniform(-2, 2)
            vy = speed * random.uniform(0.5, 1.0)
        else:
            # Default
            lane = LanePosition.CENTER
            spawn_x = 0.0
            target_x = None
            vx = 0.0
            vy = 0.0
        
        # Create obstacle
        obstacle = Obstacle(
            id=self.next_id,
            obstacle_type=obs_type,
            x=spawn_x,
            y=spawn_y,
            vx=vx,
            vy=vy,
            width=template['width'],
            length=template['length'],
            height=template['height'],
            lane=lane,
            behavior=behavior,
            color=template['color'],
            target_x=target_x,
        )
        
        self.next_id += 1
        self.obstacles.append(obstacle)
        
        return obstacle
    
    def _get_lane_center(self, lane: LanePosition) -> float:
        """Get x position of lane center"""
        return (lane.value - (ROAD.num_lanes - 1) / 2) * ROAD.lane_width
    
    def update(self, dt: float, ego_y: float, ego_lane: LanePosition):
        """
        Update all obstacles
        
        Args:
            dt: Time step in seconds
            ego_y: Ego vehicle position
            ego_lane: Ego vehicle lane
        """
        # Spawn timer
        self.spawn_timer += dt
        if self.spawn_timer >= self.spawn_interval:
            self.spawn_timer = 0.0
            self.spawn_interval = random.uniform(*TRAFFIC.obstacle_spawn_interval)
            
            if random.random() < TRAFFIC.obstacle_spawn_probability:
                self.spawn_obstacle(ego_y, ego_lane)
        
        # Update each obstacle
        for obstacle in self.obstacles:
            self._update_obstacle(obstacle, dt, ego_y)
        
        # Remove inactive obstacles
        self.obstacles = [o for o in self.obstacles if o.is_active]
    
    def _update_obstacle(self, obstacle: Obstacle, dt: float, ego_y: float):
        """Update a single obstacle"""
        obstacle.time_alive += dt
        
        # Behavior-specific updates
        if obstacle.behavior == ObstacleBehavior.CROSSING:
            # Continue crossing
            obstacle.x += obstacle.vx * dt
            
            # Check if crossed the road
            road_half_width = ROAD.num_lanes * ROAD.lane_width / 2 + ROAD.shoulder_width
            if abs(obstacle.x) > road_half_width + 5:
                obstacle.is_active = False
                
        elif obstacle.behavior == ObstacleBehavior.BRAKING:
            # Gradually slow down
            obstacle.vy = max(0.0, obstacle.vy - 3.0 * dt)
            obstacle.y += obstacle.vy * dt
            
            if obstacle.vy < 0.1:
                obstacle.time_stationary += dt
                
        elif obstacle.behavior == ObstacleBehavior.ERRATIC:
            # Random direction changes
            if random.random() < 0.02:  # 2% chance per update
                obstacle.vx = random.uniform(-3, 3)
                obstacle.vy *= random.uniform(0.8, 1.2)
            
            obstacle.x += obstacle.vx * dt
            obstacle.y += obstacle.vy * dt
            
            # Keep within road
            max_x = ROAD.num_lanes * ROAD.lane_width / 2
            obstacle.x = max(-max_x, min(max_x, obstacle.x))
            
        elif obstacle.behavior == ObstacleBehavior.SAME_DIRECTION:
            obstacle.y += obstacle.vy * dt
            
        elif obstacle.behavior == ObstacleBehavior.STATIC:
            obstacle.time_stationary += dt
        
        # Deactivate if too far behind or ahead
        distance = obstacle.y - ego_y
        if distance < -30 or distance > 250:
            obstacle.is_active = False
    
    def get_nearest_obstacle(self, ego_y: float, ego_lane: LanePosition,
                            check_all_lanes: bool = False) -> Optional[Obstacle]:
        """
        Get the nearest obstacle in path
        
        Args:
            ego_y: Ego vehicle position
            ego_lane: Ego vehicle lane
            check_all_lanes: Check obstacles in all lanes
        """
        nearest = None
        min_distance = float('inf')
        
        lane_x = self._get_lane_center(ego_lane)
        lane_width = ROAD.lane_width
        
        for obstacle in self.obstacles:
            if not obstacle.is_active:
                continue
                
            distance = obstacle.y - ego_y
            if distance < 0:
                continue  # Behind us
            
            # Check if in our lane (or crossing into it)
            if check_all_lanes:
                in_path = True
            else:
                obstacle_left = obstacle.x - obstacle.width / 2
                obstacle_right = obstacle.x + obstacle.width / 2
                lane_left = lane_x - lane_width / 2
                lane_right = lane_x + lane_width / 2
                
                in_path = not (obstacle_right < lane_left or obstacle_left > lane_right)
            
            if in_path and distance < min_distance:
                min_distance = distance
                nearest = obstacle
        
        return nearest
    
    def get_obstacles_in_lane(self, lane: LanePosition, ego_y: float,
                              range_ahead: float = 150.0) -> List[Obstacle]:
        """Get all obstacles in a specific lane within range"""
        lane_x = self._get_lane_center(lane)
        lane_half = ROAD.lane_width / 2
        
        result = []
        for obstacle in self.obstacles:
            if not obstacle.is_active:
                continue
            
            distance = obstacle.y - ego_y
            if distance < 0 or distance > range_ahead:
                continue
            
            if lane_x - lane_half - 0.5 <= obstacle.x <= lane_x + lane_half + 0.5:
                result.append(obstacle)
        
        return result
    
    def is_lane_clear(self, lane: LanePosition, ego_y: float,
                      required_gap: float = 50.0) -> bool:
        """Check if a lane is clear for lane change"""
        obstacles = self.get_obstacles_in_lane(lane, ego_y - 20, required_gap + 40)
        return len(obstacles) == 0
    
    def get_all_obstacles(self) -> List[Obstacle]:
        """Get all active obstacles"""
        return [o for o in self.obstacles if o.is_active]
    
    def clear_all(self):
        """Remove all obstacles"""
        self.obstacles.clear()
        self.next_id = 0
        self.spawn_timer = 0.0
