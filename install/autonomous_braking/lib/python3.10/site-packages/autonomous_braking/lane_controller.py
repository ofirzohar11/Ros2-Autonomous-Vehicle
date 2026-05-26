"""
═══════════════════════════════════════════════════════════════════════════════
    LANE CONTROLLER MODULE
    Intelligent lane change decision making for AEB system
    
    Features:
    - Safety-first lane change decisions
    - Obstacle avoidance
    - Traffic-aware maneuvering
═══════════════════════════════════════════════════════════════════════════════
"""

import math
from dataclasses import dataclass
from typing import Optional, List, Tuple
from enum import Enum

from config import ROAD, SAFETY, PHYSICS, LanePosition, VehicleStatus
from obstacle_manager import Obstacle, ObstacleManager


class LaneChangeDecision(Enum):
    """Lane change decision types"""
    NONE = "none"
    LEFT = "left"
    RIGHT = "right"
    EMERGENCY_LEFT = "emergency_left"
    EMERGENCY_RIGHT = "emergency_right"
    BRAKE_ONLY = "brake_only"


@dataclass
class LaneStatus:
    """Status of a single lane"""
    lane: LanePosition
    is_clear: bool
    nearest_obstacle_distance: float
    nearest_obstacle_velocity: float
    time_to_collision: float
    safety_score: float  # 0.0 (dangerous) to 1.0 (safe)


@dataclass
class LaneChangeResult:
    """Result of lane change decision"""
    decision: LaneChangeDecision
    target_lane: Optional[LanePosition]
    urgency: float  # 0.0 (not urgent) to 1.0 (critical)
    reason: str
    alternative: Optional[LaneChangeDecision]


class LaneController:
    """
    Intelligent lane change controller
    
    Decision hierarchy:
    1. If collision imminent and lane change possible -> Emergency lane change
    2. If obstacle blocking and stationary -> Attempt lane change
    3. If slower vehicle ahead -> Consider lane change for efficiency
    4. Otherwise -> Stay in lane
    """
    
    def __init__(self, obstacle_manager: ObstacleManager):
        self.obstacle_manager = obstacle_manager
        
        # Lane change parameters
        self.min_lane_change_gap = 30.0  # meters
        self.safe_lane_change_gap = 50.0  # meters
        self.look_behind_distance = 30.0  # meters
        self.stationary_threshold = 1.0  # m/s
        self.stationary_wait_time = 2.0  # seconds before attempting lane change
        
        # State
        self.obstacle_wait_timer = 0.0
        self.last_obstacle_id: Optional[int] = None
        
    def evaluate_lane(self, lane: LanePosition, ego_y: float,
                      ego_speed: float) -> LaneStatus:
        """
        Evaluate the safety status of a lane
        
        Args:
            lane: Lane to evaluate
            ego_y: Ego vehicle position
            ego_speed: Ego vehicle speed (m/s)
        """
        obstacles_ahead = self.obstacle_manager.get_obstacles_in_lane(
            lane, ego_y, self.safe_lane_change_gap + 50
        )
        
        # Also check behind for lane change safety
        obstacles_behind = self.obstacle_manager.get_obstacles_in_lane(
            lane, ego_y - self.look_behind_distance, self.look_behind_distance
        )
        
        # Find nearest obstacle ahead
        nearest_dist = float('inf')
        nearest_vel = 0.0
        
        for obs in obstacles_ahead:
            dist = obs.y - ego_y
            if 0 < dist < nearest_dist:
                nearest_dist = dist
                nearest_vel = obs.vy
        
        # Calculate TTC
        relative_vel = ego_speed - nearest_vel
        if relative_vel > 0 and nearest_dist < float('inf'):
            ttc = nearest_dist / relative_vel
        else:
            ttc = float('inf')
        
        # Check if clear for lane change
        is_clear = (nearest_dist > self.min_lane_change_gap and
                   len(obstacles_behind) == 0)
        
        # Calculate safety score
        safety_score = self._calculate_safety_score(
            nearest_dist, ttc, len(obstacles_ahead), len(obstacles_behind)
        )
        
        return LaneStatus(
            lane=lane,
            is_clear=is_clear,
            nearest_obstacle_distance=nearest_dist,
            nearest_obstacle_velocity=nearest_vel,
            time_to_collision=ttc,
            safety_score=safety_score
        )
    
    def _calculate_safety_score(self, distance: float, ttc: float,
                                 num_ahead: int, num_behind: int) -> float:
        """Calculate a safety score from 0.0 to 1.0"""
        score = 1.0
        
        # Distance factor
        if distance < SAFETY.emergency_distance:
            score *= 0.1
        elif distance < SAFETY.danger_distance:
            score *= 0.3
        elif distance < SAFETY.warning_distance:
            score *= 0.5
        elif distance < SAFETY.caution_distance:
            score *= 0.7
        elif distance < SAFETY.safe_distance:
            score *= 0.9
        
        # TTC factor
        if ttc < 1.0:
            score *= 0.2
        elif ttc < 2.0:
            score *= 0.5
        elif ttc < 3.0:
            score *= 0.7
        
        # Obstacle count factor
        score *= max(0.5, 1.0 - num_ahead * 0.1)
        score *= max(0.7, 1.0 - num_behind * 0.15)
        
        return max(0.0, min(1.0, score))
    
    def decide_lane_change(self, ego_y: float, ego_speed: float,
                           current_lane: LanePosition,
                           current_status: VehicleStatus,
                           nearest_obstacle: Optional[Obstacle]) -> LaneChangeResult:
        """
        Decide whether to change lanes
        
        Args:
            ego_y: Ego vehicle position
            ego_speed: Ego vehicle speed (m/s)
            current_lane: Current lane
            current_status: Current safety status
            nearest_obstacle: Nearest obstacle in path
        """
        # Evaluate all lanes
        lane_statuses = {}
        for lane in LanePosition:
            lane_statuses[lane] = self.evaluate_lane(lane, ego_y, ego_speed)
        
        current_lane_status = lane_statuses[current_lane]
        
        # Check if we need to track a stationary obstacle
        if nearest_obstacle:
            is_stationary = abs(nearest_obstacle.vy) < self.stationary_threshold
            
            if is_stationary:
                if nearest_obstacle.id == self.last_obstacle_id:
                    self.obstacle_wait_timer += 0.05  # Assuming 20Hz
                else:
                    self.last_obstacle_id = nearest_obstacle.id
                    self.obstacle_wait_timer = 0.0
            else:
                self.obstacle_wait_timer = 0.0
                self.last_obstacle_id = None
        else:
            self.obstacle_wait_timer = 0.0
            self.last_obstacle_id = None
        
        # Emergency situation - collision imminent
        if current_status in [VehicleStatus.EMERGENCY, VehicleStatus.DANGER]:
            return self._handle_emergency(
                current_lane, lane_statuses, ego_speed
            )
        
        # Stationary obstacle blocking - need to go around
        if (nearest_obstacle and 
            self.obstacle_wait_timer >= self.stationary_wait_time):
            return self._handle_stationary_obstacle(
                current_lane, lane_statuses, nearest_obstacle
            )
        
        # Warning - consider preventive lane change
        if current_status == VehicleStatus.WARNING:
            return self._handle_warning(
                current_lane, lane_statuses, ego_speed
            )
        
        # Normal driving - no lane change needed
        return LaneChangeResult(
            decision=LaneChangeDecision.NONE,
            target_lane=None,
            urgency=0.0,
            reason="Path clear, continuing in current lane",
            alternative=None
        )
    
    def _handle_emergency(self, current_lane: LanePosition,
                          lane_statuses: dict,
                          ego_speed: float) -> LaneChangeResult:
        """Handle emergency situation requiring immediate action"""
        
        # Check adjacent lanes
        possible_escapes = []
        
        # Left lane
        if current_lane.value > 0:
            left_lane = LanePosition(current_lane.value - 1)
            left_status = lane_statuses[left_lane]
            if left_status.is_clear:
                possible_escapes.append((left_lane, left_status.safety_score, 'left'))
        
        # Right lane
        if current_lane.value < ROAD.num_lanes - 1:
            right_lane = LanePosition(current_lane.value + 1)
            right_status = lane_statuses[right_lane]
            if right_status.is_clear:
                possible_escapes.append((right_lane, right_status.safety_score, 'right'))
        
        if possible_escapes:
            # Choose safest option
            possible_escapes.sort(key=lambda x: x[1], reverse=True)
            best = possible_escapes[0]
            
            decision = (LaneChangeDecision.EMERGENCY_LEFT if best[2] == 'left'
                       else LaneChangeDecision.EMERGENCY_RIGHT)
            
            return LaneChangeResult(
                decision=decision,
                target_lane=best[0],
                urgency=1.0,
                reason=f"EMERGENCY: Collision imminent, escaping to {best[2]}",
                alternative=LaneChangeDecision.BRAKE_ONLY
            )
        
        # No escape route - brake only
        return LaneChangeResult(
            decision=LaneChangeDecision.BRAKE_ONLY,
            target_lane=None,
            urgency=1.0,
            reason="EMERGENCY: No safe lane available, maximum braking",
            alternative=None
        )
    
    def _handle_stationary_obstacle(self, current_lane: LanePosition,
                                     lane_statuses: dict,
                                     obstacle: Obstacle) -> LaneChangeResult:
        """Handle stationary obstacle blocking the path"""
        
        # Check adjacent lanes
        options = []
        
        # Prefer the lane with most space
        if current_lane.value > 0:
            left_lane = LanePosition(current_lane.value - 1)
            left_status = lane_statuses[left_lane]
            if left_status.nearest_obstacle_distance > self.min_lane_change_gap:
                options.append((left_lane, left_status.safety_score, 'left'))
        
        if current_lane.value < ROAD.num_lanes - 1:
            right_lane = LanePosition(current_lane.value + 1)
            right_status = lane_statuses[right_lane]
            if right_status.nearest_obstacle_distance > self.min_lane_change_gap:
                options.append((right_lane, right_status.safety_score, 'right'))
        
        if options:
            options.sort(key=lambda x: x[1], reverse=True)
            best = options[0]
            
            decision = (LaneChangeDecision.LEFT if best[2] == 'left'
                       else LaneChangeDecision.RIGHT)
            
            return LaneChangeResult(
                decision=decision,
                target_lane=best[0],
                urgency=0.7,
                reason=f"Stationary {obstacle.obstacle_type.value} blocking, changing to {best[2]}",
                alternative=LaneChangeDecision.BRAKE_ONLY
            )
        
        return LaneChangeResult(
            decision=LaneChangeDecision.BRAKE_ONLY,
            target_lane=None,
            urgency=0.5,
            reason="Stationary obstacle, no safe lane change available",
            alternative=None
        )
    
    def _handle_warning(self, current_lane: LanePosition,
                        lane_statuses: dict,
                        ego_speed: float) -> LaneChangeResult:
        """Handle warning situation - preventive action"""
        
        current_status = lane_statuses[current_lane]
        
        # Check if adjacent lanes are better
        better_lanes = []
        
        for lane, status in lane_statuses.items():
            if lane == current_lane:
                continue
            
            # Lane must be adjacent
            if abs(lane.value - current_lane.value) != 1:
                continue
            
            # Lane should be significantly better
            if status.safety_score > current_status.safety_score + 0.2:
                if status.is_clear:
                    better_lanes.append((lane, status.safety_score))
        
        if better_lanes:
            better_lanes.sort(key=lambda x: x[1], reverse=True)
            best_lane = better_lanes[0][0]
            
            direction = 'left' if best_lane.value < current_lane.value else 'right'
            decision = (LaneChangeDecision.LEFT if direction == 'left'
                       else LaneChangeDecision.RIGHT)
            
            return LaneChangeResult(
                decision=decision,
                target_lane=best_lane,
                urgency=0.5,
                reason=f"Preventive lane change to {direction} for better safety",
                alternative=LaneChangeDecision.BRAKE_ONLY
            )
        
        return LaneChangeResult(
            decision=LaneChangeDecision.BRAKE_ONLY,
            target_lane=None,
            urgency=0.3,
            reason="Warning: Reducing speed for safety",
            alternative=None
        )
    
    def get_lane_visualization_data(self, ego_y: float, ego_speed: float) -> dict:
        """Get lane status data for visualization"""
        data = {}
        
        for lane in LanePosition:
            status = self.evaluate_lane(lane, ego_y, ego_speed)
            data[lane.name] = {
                'is_clear': status.is_clear,
                'distance': status.nearest_obstacle_distance,
                'ttc': status.time_to_collision,
                'safety': status.safety_score,
            }
        
        return data
