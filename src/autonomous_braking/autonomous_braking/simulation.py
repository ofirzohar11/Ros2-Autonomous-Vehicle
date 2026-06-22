#!/usr/bin/env python3
"""
AUTONOMOUS EMERGENCY BRAKING - SIMULATION ENGINE
Handles: Vehicle dynamics, obstacles, LIDAR, AEB decisions, Scenario Events
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float64, Int32
import json
import math
import time
import random
import numpy as np
from typing import List, Optional, Tuple, Dict
from dataclasses import dataclass, field

from autonomous_braking.config import (
    PHYSICS, PID, BRAKE_PID, SAFETY, LIDAR_CFG, ROAD, TRAFFIC,
    ObstacleType, VehicleStatus, LanePosition, ObstacleBehavior, Topics
)
from autonomous_braking.pid_controller import PIDController


# ═══════════════════════════════════════════════════════════════
#  DATA STRUCTURES
# ═══════════════════════════════════════════════════════════════

@dataclass
class VehicleState:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    lane: LanePosition = LanePosition.CENTER
    width: float = 1.8
    length: float = 4.5
    height: float = 1.5


@dataclass
class Obstacle:
    id: int
    obstacle_type: ObstacleType
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    width: float = 1.8
    length: float = 4.5
    height: float = 1.5
    lane: LanePosition = LanePosition.CENTER
    behavior: ObstacleBehavior = ObstacleBehavior.SAME_DIRECTION
    color: Tuple[int, int, int] = (200, 50, 50)
    active: bool = True
    behavior_timer: float = 0.0

    def to_dict(self):
        return {
            'id': self.id,
            'type': self.obstacle_type.value,
            'x': round(self.x, 2),
            'y': round(self.y, 2),
            'vx': round(self.vx, 2),
            'vy': round(self.vy, 2),
            'width': self.width,
            'length': self.length,
            'height': self.height,
            'lane': self.lane.value,
            'behavior': self.behavior.value,
            'color': list(self.color),
            'active': self.active
        }


@dataclass
class LidarPoint:
    angle: float
    distance: float
    layer: int
    x: float = 0.0
    y: float = 0.0
    obstacle_id: int = -1


# ═══════════════════════════════════════════════════════════════
#  OBSTACLE TEMPLATES
# ═══════════════════════════════════════════════════════════════

OBSTACLE_TEMPLATES = {
    ObstacleType.CAR: {
        'width': 1.8, 'length': 4.5, 'height': 1.5,
        'color': (200, 50, 50),
        'behaviors': [ObstacleBehavior.SAME_DIRECTION, ObstacleBehavior.BRAKING],
        'speed_range': (60, 100),
    },
    ObstacleType.PEDESTRIAN: {
        'width': 0.5, 'length': 0.5, 'height': 1.7,
        'color': (100, 100, 200),
        'behaviors': [ObstacleBehavior.CROSSING],
        'speed_range': (3, 6),
    },
    ObstacleType.CHILD: {
        'width': 0.4, 'length': 0.4, 'height': 1.2,
        'color': (200, 100, 200),
        'behaviors': [ObstacleBehavior.CROSSING, ObstacleBehavior.ERRATIC],
        'speed_range': (4, 8),
    },
    ObstacleType.BALL: {
        'width': 0.3, 'length': 0.3, 'height': 0.3,
        'color': (255, 200, 0),
        'behaviors': [ObstacleBehavior.CROSSING],
        'speed_range': (5, 15),
    },
    ObstacleType.FALLEN_TREE: {
        'width': 3.0, 'length': 1.5, 'height': 0.8,
        'color': (101, 67, 33),
        'behaviors': [ObstacleBehavior.STATIC],
        'speed_range': (0, 0),
    },
    ObstacleType.MOTORCYCLE: {
        'width': 0.8, 'length': 2.2, 'height': 1.4,
        'color': (100, 100, 200),
        'behaviors': [ObstacleBehavior.SAME_DIRECTION, ObstacleBehavior.ERRATIC],
        'speed_range': (70, 120),
    },
    ObstacleType.BICYCLE: {
        'width': 0.6, 'length': 1.8, 'height': 1.5,
        'color': (50, 200, 100),
        'behaviors': [ObstacleBehavior.SAME_DIRECTION, ObstacleBehavior.CROSSING],
        'speed_range': (15, 30),
    },
    ObstacleType.ANIMAL: {
        'width': 0.5, 'length': 0.8, 'height': 0.5,
        'color': (139, 90, 43),
        'behaviors': [ObstacleBehavior.CROSSING, ObstacleBehavior.ERRATIC],
        'speed_range': (10, 40),
    },
}


def get_lane_center_x(lane: LanePosition) -> float:
    return (lane.value - (ROAD.num_lanes - 1) / 2) * ROAD.lane_width


# ═══════════════════════════════════════════════════════════════
#  SCENARIO SYSTEM
# ═══════════════════════════════════════════════════════════════

MAX_SCENARIO_DURATION = 18.0   # force-end any scenario after this many seconds

# Human-readable names shown on dashboard
SCENARIO_NAMES = {
    'front_brake': 'Front Vehicle Braking',
    'tree_right':  'Fallen Tree — Right Side',
    'tree_left':   'Fallen Tree — Left Side',
    'pedestrian':  'Pedestrian Crossing',
}


# ═══════════════════════════════════════════════════════════════
#  WORLD SIMULATION NODE
# ═══════════════════════════════════════════════════════════════

class WorldSimulation(Node):
    def __init__(self):
        super().__init__('world_simulation')

        # Ego vehicle
        self.ego = VehicleState()
        self.ego.lane = LanePosition.CENTER
        self.ego.x = get_lane_center_x(LanePosition.CENTER)
        self.ego.vy = PHYSICS.cruise_speed_kmh / 3.6

        # AEB State
        self.status = VehicleStatus.SAFE
        self.brake_intensity = 0.0
        self.target_speed = PHYSICS.cruise_speed_kmh / 3.6
        self.nearest_distance = 999.0
        self.nearest_type = ""
        self.ttc = 999.0

        # Longitudinal speed controller (PID): tracks target_speed by
        # commanding an acceleration. Emergency braking overrides it.
        self.speed_pid = PIDController(
            kp=PID.kp, ki=PID.ki, kd=PID.kd,
            output_min=PID.output_min, output_max=PID.output_max,
            integral_limit=PID.integral_limit,
        )
        self.pid_output = 0.0      # last acceleration command (m/s²)
        self.speed_error = 0.0     # last speed error (m/s)

        # Braking controller (PID): tracks a safe following gap and turns
        # the distance error into a brake intensity (0..1). The emergency
        # range / critical TTC still hard-override it with full braking.
        self.brake_pid = PIDController(
            kp=BRAKE_PID.kp, ki=BRAKE_PID.ki, kd=BRAKE_PID.kd,
            output_min=BRAKE_PID.output_min, output_max=BRAKE_PID.output_max,
            integral_limit=BRAKE_PID.integral_limit,
        )
        self.desired_gap = 0.0     # last computed safe following distance (m)
        self.gap_error = 0.0       # last distance error (m): desired − actual

        # Lane change state
        self.is_changing_lane = False
        self.lane_change_progress = 0.0
        self.lane_change_from = LanePosition.CENTER
        self.lane_change_to = LanePosition.CENTER
        self.lane_change_start_x = 0.0
        self.lane_change_target_x = 0.0

        # Obstacles
        self.obstacles: List[Obstacle] = []
        self.next_obstacle_id = 0

        # Traffic vehicles (background)
        self.traffic_vehicles: List[Obstacle] = []
        self._init_traffic()

        # LIDAR data
        self.lidar_points: List[Dict] = []

        # ── Scenario system ─────────────────────────────────────
        # Random driving events are triggered every 6–11 seconds.
        # One of four scenarios is selected: front_brake, tree_right,
        # tree_left, or pedestrian.  Each has a clear start/active/end phase.
        self.active_scenario: Optional[str] = None   # key into SCENARIO_NAMES
        self.scenario_state = 'idle'                 # 'idle' | 'active'
        self.scenario_idle_timer = 0.0               # seconds since last scenario ended
        self.next_scenario_time = random.uniform(6.0, 11.0)
        self.scenario_phase_timer = 0.0              # seconds inside current scenario
        self.scenario_obstacle_ids: List[int] = []  # IDs of scenario-spawned obstacles
        self.decision_text = "Safe: road clear"

        # Timer used for "return to center" after obstacle avoidance
        self.safe_timer = 0.0

        # Human-like micro-speed variation while cruising
        self.human_speed_timer = 0.0
        self.human_speed_next  = random.uniform(4.0, 9.0)
        self.human_speed_offset = 0.0   # m/s offset on top of cruise speed

        # Statistics
        self.total_distance = 0.0
        self.braking_events = 0
        self.lane_changes = 0
        self.start_time = time.time()

        # Publishers
        self.pub_state = self.create_publisher(String, Topics.FULL_STATE, 10)
        self.pub_speed = self.create_publisher(Float64, Topics.EGO_SPEED, 10)
        self.pub_lane = self.create_publisher(Int32, Topics.EGO_LANE, 10)

        # 50 Hz simulation timer
        self.dt = 0.02
        self.create_timer(self.dt, self.simulation_step)

        self.get_logger().info('World Simulation started — AEB System Active')

    def _init_traffic(self):
        """Spawn initial background traffic vehicles in random lanes."""
        num_traffic = random.randint(
            TRAFFIC.min_traffic_vehicles,
            TRAFFIC.max_traffic_vehicles
        )
        used_positions = []

        for _ in range(num_traffic):
            lane = LanePosition(random.randint(0, ROAD.num_lanes - 1))
            for _ in range(10):
                y_offset = random.uniform(18, 55)
                too_close = any(
                    prev_lane == lane and abs(prev_y - y_offset) < TRAFFIC.min_vehicle_spacing
                    for prev_lane, prev_y in used_positions
                )
                if not too_close:
                    break
                lane = LanePosition(random.randint(0, ROAD.num_lanes - 1))

            used_positions.append((lane, y_offset))

            speed_var = random.uniform(-TRAFFIC.traffic_speed_variance, TRAFFIC.traffic_speed_variance)
            speed = (PHYSICS.cruise_speed_kmh / 3.6) * (1.0 + speed_var)

            colors = [
                (200, 50, 50), (50, 50, 200), (200, 200, 200),
                (50, 200, 50), (200, 150, 50), (150, 50, 150),
                (200, 200, 50), (100, 100, 100), (220, 180, 180)
            ]

            obs = Obstacle(
                id=self._get_next_id(),
                obstacle_type=ObstacleType.CAR,
                x=get_lane_center_x(lane),
                y=self.ego.y + y_offset,
                vy=speed,
                width=1.8, length=4.5, height=1.5,
                lane=lane,
                behavior=ObstacleBehavior.SAME_DIRECTION,
                color=random.choice(colors),
            )
            self.traffic_vehicles.append(obs)

    def _get_next_id(self) -> int:
        self.next_obstacle_id += 1
        return self.next_obstacle_id

    # ─── MAIN SIMULATION STEP ────────────────────────────────

    def simulation_step(self):
        """Main simulation loop at 50 Hz."""
        self._update_traffic()
        self._update_obstacles()
        self._update_scenario()        # manage active scenario lifecycle
        self._try_trigger_scenario()   # schedule next scenario when idle
        self._run_lidar_scan()
        self._aeb_decision()
        self._update_ego_physics()

        if self.is_changing_lane:
            self._update_lane_change()

        self._publish_state()
        self.total_distance += self.ego.vy * self.dt

    # ─── TRAFFIC UPDATE ──────────────────────────────────────

    def _update_traffic(self):
        for tv in self.traffic_vehicles:
            tv.y += tv.vy * self.dt

            tv.behavior_timer += self.dt
            if tv.behavior_timer > 5.0:
                tv.behavior_timer = 0.0
                speed_change = random.uniform(-0.3, 0.3)
                base = PHYSICS.cruise_speed_kmh / 3.6
                tv.vy = max(base * 0.7, min(base * 1.2, tv.vy + speed_change))

            if tv.y - self.ego.y > 70 or self.ego.y - tv.y > 12:
                tv.y = self.ego.y + random.uniform(22, 55)
                new_lane = LanePosition(random.randint(0, ROAD.num_lanes - 1))
                tv.lane = new_lane
                tv.x = get_lane_center_x(new_lane)
                speed_var = random.uniform(-TRAFFIC.traffic_speed_variance, TRAFFIC.traffic_speed_variance)
                tv.vy = (PHYSICS.cruise_speed_kmh / 3.6) * (1.0 + speed_var)

    # ─── OBSTACLE UPDATE ─────────────────────────────────────

    def _update_obstacles(self):
        for obs in self.obstacles[:]:
            if not obs.active:
                self.obstacles.remove(obs)
                continue

            obs.x += obs.vx * self.dt
            obs.y += obs.vy * self.dt

            if obs.behavior == ObstacleBehavior.ERRATIC:
                obs.behavior_timer += self.dt
                if obs.behavior_timer > 1.0:
                    obs.behavior_timer = 0.0
                    obs.vx += random.uniform(-1, 1)
                    obs.vy += random.uniform(-0.5, 0.5)

            rel_y = obs.y - self.ego.y
            if rel_y < -20 or rel_y > 250 or abs(obs.x) > 20:
                obs.active = False

    # ─── SCENARIO SYSTEM ─────────────────────────────────────

    def _try_trigger_scenario(self):
        """
        Increment idle timer and trigger the next random scenario once
        the interval (6–11 s) has elapsed and the road is not already busy.
        """
        if self.scenario_state != 'idle':
            return

        self.scenario_idle_timer += self.dt
        if self.scenario_idle_timer < self.next_scenario_time:
            return

        # Don't pile on obstacles when the road is already crowded
        if len(self.obstacles) >= 2:
            return

        self.scenario_idle_timer = 0.0
        self._trigger_random_scenario()

    def _trigger_random_scenario(self):
        """
        Select and initialise one of the four driving scenarios:
          A. front_brake  – vehicle ahead brakes hard
          B. tree_right   – tree falls from right, blocks right+ego lane
          C. tree_left    – tree falls from left, blocks left+ego lane
          D. pedestrian   – child crosses all lanes

        Selection is restricted to scenarios that are physically solvable
        given the ego's current lane position.
        """
        ego_lane = self.ego.lane

        # Build list of valid choices based on available escape directions.
        # Weights give front_brake / pedestrian a higher share than tree scenarios.
        WEIGHTS = {
            'front_brake': 35,
            'pedestrian':  35,
            'tree_right':  15,
            'tree_left':   15,
        }
        valid_scenarios = ['front_brake', 'pedestrian']
        if ego_lane.value > 0:               # ego can change left  → tree from right
            valid_scenarios.append('tree_right')
        if ego_lane.value < ROAD.num_lanes - 1:  # ego can change right → tree from left
            valid_scenarios.append('tree_left')

        ws = [WEIGHTS[s] for s in valid_scenarios]
        scenario = random.choices(valid_scenarios, weights=ws, k=1)[0]
        self.active_scenario = scenario
        self.scenario_state = 'active'
        self.scenario_phase_timer = 0.0
        self.scenario_obstacle_ids = []

        spawn_dist = 30.0   # meters ahead of ego when spawned

        # ── Scenario A: front vehicle brakes hard ────────────
        if scenario == 'front_brake':
            obs = Obstacle(
                id=self._get_next_id(),
                obstacle_type=ObstacleType.CAR,
                x=get_lane_center_x(ego_lane),
                y=self.ego.y + spawn_dist,
                vy=self.ego.vy * 0.85,       # slightly slower to start
                width=1.8, length=4.5, height=1.5,
                lane=ego_lane,
                behavior=ObstacleBehavior.BRAKING,
                color=(255, 60, 60),
            )
            self.obstacles.append(obs)
            self.scenario_obstacle_ids.append(obs.id)

        # ── Scenario B: fallen tree from right ───────────────
        elif scenario == 'tree_right':
            # Place tree between ego's lane and the lane to the right
            right_lane = LanePosition(min(ROAD.num_lanes - 1, ego_lane.value + 1))
            tree_x = (get_lane_center_x(ego_lane) + get_lane_center_x(right_lane)) / 2.0
            obs = Obstacle(
                id=self._get_next_id(),
                obstacle_type=ObstacleType.FALLEN_TREE,
                x=tree_x,
                y=self.ego.y + spawn_dist,
                vx=0.0, vy=0.0,
                # Blocks the ego lane and the right lane it sits between;
                # the ego can still clear it by changing one lane left.
                width=ROAD.lane_width * 1.1,
                length=2.5, height=1.2,
                lane=ego_lane,
                behavior=ObstacleBehavior.STATIC,
                color=(90, 55, 20),
            )
            self.obstacles.append(obs)
            self.scenario_obstacle_ids.append(obs.id)

        # ── Scenario C: fallen tree from left ────────────────
        elif scenario == 'tree_left':
            # Place tree between ego's lane and the lane to the left
            left_lane = LanePosition(max(0, ego_lane.value - 1))
            tree_x = (get_lane_center_x(ego_lane) + get_lane_center_x(left_lane)) / 2.0
            obs = Obstacle(
                id=self._get_next_id(),
                obstacle_type=ObstacleType.FALLEN_TREE,
                x=tree_x,
                y=self.ego.y + spawn_dist,
                vx=0.0, vy=0.0,
                # Blocks the ego lane and the left lane it sits between;
                # the ego can still clear it by changing one lane right.
                width=ROAD.lane_width * 1.1,
                length=2.5, height=1.2,
                lane=ego_lane,
                behavior=ObstacleBehavior.STATIC,
                color=(90, 55, 20),
            )
            self.obstacles.append(obs)
            self.scenario_obstacle_ids.append(obs.id)

        # ── Scenario D: pedestrian crossing ──────────────────
        elif scenario == 'pedestrian':
            side = random.choice([-1, 1])   # -1 = from left side, +1 = from right side
            road_half = ROAD.num_lanes * ROAD.lane_width / 2.0
            # Start child already inside the road (~2 m from centre) so it is
            # detected immediately and ego has enough time to brake fully.
            spawn_x = side * road_half * 0.36   # ≈ ±2.0 m from road centre
            cross_speed = random.uniform(4, 7) / 3.6  # walking speed
            obs = Obstacle(
                id=self._get_next_id(),
                obstacle_type=ObstacleType.CHILD,
                x=spawn_x,
                y=self.ego.y + 28.0,
                vx=-side * cross_speed,       # walks toward opposite side
                vy=0.0,
                width=0.5, length=0.5, height=1.2,
                lane=LanePosition.CENTER,
                behavior=ObstacleBehavior.CROSSING,
                color=(255, 150, 50),
            )
            self.obstacles.append(obs)
            self.scenario_obstacle_ids.append(obs.id)

        self.get_logger().info(f'Scenario: {SCENARIO_NAMES[scenario]}')

    def _update_scenario(self):
        """
        Run per-step logic for the active scenario and detect its completion.
        Each scenario has a clear active phase and an end condition.
        """
        if self.scenario_state != 'active':
            return

        self.scenario_phase_timer += self.dt
        scenario = self.active_scenario

        # Safety valve: force-end any scenario that runs longer than the max
        # duration (e.g. ego stopped in front of a tree with no escape route).
        if self.scenario_phase_timer > MAX_SCENARIO_DURATION:
            for obs in self.obstacles:
                if obs.id in self.scenario_obstacle_ids:
                    obs.active = False
            self._end_scenario()
            return

        # Get still-active obstacles that belong to this scenario
        active_obs = [o for o in self.obstacles
                      if o.id in self.scenario_obstacle_ids and o.active]

        if scenario == 'front_brake' and active_obs:
            obs = active_obs[0]
            # After 1.5 s: apply hard braking to simulate emergency stop
            if self.scenario_phase_timer > 1.5:
                obs.vy = max(0.0, obs.vy - 7.0 * self.dt)
            # Scenario ends when ego has overtaken (or stopped) the car
            if obs.y - self.ego.y < -15.0:
                obs.active = False
                self._end_scenario()

        elif scenario in ('tree_right', 'tree_left') and active_obs:
            # Static tree — end scenario once ego has safely passed it
            obs = active_obs[0]
            if obs.y - self.ego.y < -25.0:
                obs.active = False
                self._end_scenario()

        elif scenario == 'pedestrian' and active_obs:
            obs = active_obs[0]
            road_half = ROAD.num_lanes * ROAD.lane_width / 2.0
            # Child has cleared all lanes
            if abs(obs.x) > road_half + 3.5:
                obs.active = False
                self._end_scenario()

        # Fallback: all scenario obstacles were removed externally
        elif not active_obs:
            self._end_scenario()

    def _end_scenario(self):
        """Mark the current scenario complete and schedule the next one."""
        if self.active_scenario:
            self.get_logger().info(
                f'Scenario complete: {SCENARIO_NAMES.get(self.active_scenario, self.active_scenario)}'
            )
        self.active_scenario = None
        self.scenario_state = 'idle'
        self.scenario_obstacle_ids = []
        self.scenario_phase_timer = 0.0
        self.scenario_idle_timer = 0.0
        self.next_scenario_time = random.uniform(6.0, 11.0)

    def _get_decision_text(self) -> str:
        """Return a short human-readable description of the current AEB action."""
        scenario = self.active_scenario
        status = self.status
        dist = self.nearest_distance
        obj = self.nearest_type or 'obstacle'

        if self.is_changing_lane:
            target = self.lane_change_to.name.lower()
            if scenario == 'tree_right':
                return "Lane change left: tree blocking path"
            if scenario == 'tree_left':
                return "Lane change right: tree blocking path"
            return f"Lane change {target}: obstacle ahead"

        if status == VehicleStatus.STOPPED:
            if scenario == 'pedestrian':
                return "Stopped: pedestrian crossing — waiting"
            return f"Stopped: {obj} at {dist:.0f}m"

        if status == VehicleStatus.EMERGENCY:
            if scenario == 'pedestrian':
                return "EMERGENCY STOP: pedestrian on road"
            return f"EMERGENCY BRAKE: {obj} at {dist:.0f}m"

        if status == VehicleStatus.DANGER:
            if scenario == 'front_brake':
                return "Braking hard: front vehicle stopped"
            return f"Braking hard: {obj} {dist:.0f}m ahead"

        if status == VehicleStatus.WARNING:
            return f"Braking: {obj} detected {dist:.0f}m ahead"

        if status == VehicleStatus.CAUTION:
            return f"Caution: {obj} {dist:.0f}m ahead"

        return "Safe: road clear"

    # ─── LIDAR SCAN (FORWARD-FACING) ─────────────────────────

    def _run_lidar_scan(self):
        """
        Forward-facing LIDAR: scans ±FOV/2 degrees from straight ahead.
        In our coordinate system, forward = +Y, angle 0° = straight ahead.
        """
        self.lidar_points = []
        all_objects = self.traffic_vehicles + self.obstacles

        half_fov = LIDAR_CFG.horizontal_fov / 2.0
        step = max(1, int(LIDAR_CFG.horizontal_fov / LIDAR_CFG.num_beams))

        for angle_offset in range(int(-half_fov), int(half_fov) + 1, step):
            angle_rad = math.radians(angle_offset)
            ray_dx = math.sin(angle_rad)
            ray_dy = math.cos(angle_rad)

            closest_dist = LIDAR_CFG.max_range
            closest_id = -1

            for obj in all_objects:
                if not obj.active:
                    continue

                rel_x = obj.x - self.ego.x
                rel_y = obj.y - self.ego.y

                if rel_y < 0:
                    continue

                half_w = obj.width / 2 + 0.3
                half_l = obj.length / 2 + 0.3

                if abs(ray_dx) > 1e-6:
                    t1 = (rel_x - half_w) / ray_dx
                    t2 = (rel_x + half_w) / ray_dx
                else:
                    t1 = -1e9 if (rel_x - half_w) <= 0 <= (rel_x + half_w) else 1e9
                    t2 = t1

                if t1 > t2:
                    t1, t2 = t2, t1

                if abs(ray_dy) > 1e-6:
                    t3 = (rel_y - half_l) / ray_dy
                    t4 = (rel_y + half_l) / ray_dy
                else:
                    t3 = -1e9 if (rel_y - half_l) <= 0 <= (rel_y + half_l) else 1e9
                    t4 = t3

                if t3 > t4:
                    t3, t4 = t4, t3

                tmin = max(t1, t3)
                tmax = min(t2, t4)

                if tmin <= tmax and tmax > 0:
                    t = max(tmin, LIDAR_CFG.min_range)
                    if t < closest_dist:
                        closest_dist = t
                        closest_id = obj.id

            if closest_dist < LIDAR_CFG.max_range:
                noise = random.gauss(0, LIDAR_CFG.noise_std)
                closest_dist = max(LIDAR_CFG.min_range, closest_dist + noise)

                self.lidar_points.append({
                    'angle': angle_offset,
                    'distance': round(closest_dist, 2),
                    'x': round(closest_dist * ray_dx, 2),
                    'y': round(closest_dist * ray_dy, 2),
                    'id': closest_id
                })

    # ─── AEB DECISION ENGINE ─────────────────────────────────

    def _should_do_lane_change(self, nearest_obj: Optional['Obstacle']) -> bool:
        """
        True only when the obstacle requires a lane change rather than braking.
        - Static obstacles (fallen trees, stopped cars): always change lane.
        - Pedestrians / children / animals / balls: BRAKE and stop, never change lane.
        - Moving vehicle (same direction, still rolling): follow and brake; change
          lane only once the vehicle has nearly stopped (vy < 15 km/h).
        """
        if nearest_obj is None:
            return False
        if nearest_obj.behavior == ObstacleBehavior.STATIC:
            return True
        crossing = {ObstacleType.PEDESTRIAN, ObstacleType.CHILD,
                    ObstacleType.BALL, ObstacleType.ANIMAL}
        if nearest_obj.obstacle_type in crossing:
            return False
        # Moving vehicle: wait until it has nearly stopped before changing lane
        return nearest_obj.vy < 4.2   # ≈ 15 km/h

    def _desired_gap(self) -> float:
        """Speed-dependent safe following distance (metres).
        Grows with speed: the faster we go, the more room we keep."""
        return BRAKE_PID.min_gap + BRAKE_PID.time_headway * self.ego.vy

    def _compute_brake_pid(self, nearest_obj: Optional['Obstacle'],
                           nearest_dist: float) -> float:
        """
        Braking PID controller.

        Brakes in proportion to how far we are *inside* the desired safe
        gap (and how fast we are closing it):

            error = desired_gap − actual_distance   (positive ⇒ too close)
            brake = clamp(Kp·error + Ki·∫ + Kd·d/dt, 0, 1)

        The brake only engages when we are actually *closing in* on the
        obstacle, or when we are inside a hard minimum gap. A lead vehicle
        travelling at our own speed (closing speed ≈ 0) is handled smoothly
        by the speed PID's adaptive-cruise following instead of by braking —
        this avoids phantom/oscillating braking against same-speed traffic.

        Otherwise the controller is held reset so it reacts instantly the
        next time something starts closing.
        """
        if nearest_obj is None or nearest_dist > SAFETY.safe_distance:
            self.brake_pid.reset()
            self.desired_gap = self._desired_gap()
            self.gap_error = 0.0
            return 0.0

        self.desired_gap = self._desired_gap()
        self.gap_error = self.desired_gap - nearest_dist

        # Hard minimum gap: below this we always brake, even at matched speed.
        hard_floor = BRAKE_PID.min_gap + 0.5 * BRAKE_PID.time_headway * self.ego.vy
        closing_speed = self.ego.vy - nearest_obj.vy   # >0 ⇒ gap shrinking

        if closing_speed <= 0.3 and nearest_dist > hard_floor:
            # Stable or opening gap and not dangerously close → let the
            # speed PID keep station; hold the brake controller reset.
            self.brake_pid.reset()
            return 0.0

        return self.brake_pid.update(self.desired_gap, nearest_dist, self.dt)

    def _aeb_decision(self):
        """
        CORE AEB LOGIC:
        1. Find nearest obstacle in ego's lane (within lateral threshold)
        2. Calculate TTC
        3. Set status, brake intensity, and target speed
        4. Trigger lane change if safe
        5. Return to center lane when road clears
        """
        all_objects = self.traffic_vehicles + [o for o in self.obstacles if o.active]

        nearest_dist = 999.0
        nearest_obj = None
        nearest_type = ""

        for obj in all_objects:
            rel_y = obj.y - self.ego.y
            if rel_y < 2.0:
                continue

            # Account for the obstacle's own width so a wide object that
            # only partly intrudes into the ego lane is still detected.
            lateral_dist = abs(obj.x - self.ego.x)
            effective_lateral = max(0.0, lateral_dist - obj.width / 2.0)
            if effective_lateral < (ROAD.lane_width * 0.7):
                if rel_y < nearest_dist:
                    nearest_dist = rel_y
                    nearest_obj = obj
                    nearest_type = obj.obstacle_type.value

        self.nearest_distance = nearest_dist
        self.nearest_type = nearest_type

        if nearest_obj is not None:
            rel_speed = self.ego.vy - nearest_obj.vy
            self.ttc = (nearest_dist / rel_speed) if rel_speed > 0.5 else 999.0
        else:
            self.ttc = 999.0

        prev_status = self.status
        do_lc = self._should_do_lane_change(nearest_obj)

        # NOTE: brake_intensity is no longer set per-zone here — it is
        # computed centrally below by the braking PID. The zones decide
        # status, target_speed, and lane-change policy only.
        if nearest_dist > SAFETY.safe_distance:
            # ── SAFE: cruise with human-like micro-variation ──────────────
            self.status = VehicleStatus.SAFE
            self.human_speed_timer += self.dt
            if self.human_speed_timer >= self.human_speed_next:
                self.human_speed_timer = 0.0
                self.human_speed_next  = random.uniform(4.0, 9.0)
                self.human_speed_offset = random.uniform(-3.0, 3.0) / 3.6
            self.target_speed = PHYSICS.cruise_speed_kmh / 3.6 + self.human_speed_offset

        elif nearest_dist > SAFETY.caution_distance:
            # ── CAUTION (20–30 m) ─────────────────────────────────────────
            self.status = VehicleStatus.CAUTION
            if do_lc:
                # Stopped / static obstacle ahead → prepare lane change
                self.target_speed = PHYSICS.cruise_speed_kmh / 3.6 * 0.85
                if not self.is_changing_lane:
                    self._try_lane_change(all_objects)
            else:
                # Moving vehicle ahead → ACC following: stay slightly behind
                self.target_speed = nearest_obj.vy * 0.97

        elif nearest_dist > SAFETY.warning_distance:
            # ── WARNING (15–20 m) ─────────────────────────────────────────
            self.status = VehicleStatus.WARNING
            if do_lc:
                self.target_speed = PHYSICS.cruise_speed_kmh / 3.6 * 0.55
                if not self.is_changing_lane:
                    self._try_lane_change(all_objects)
            else:
                # Moving vehicle → follow closer behind
                self.target_speed = nearest_obj.vy * 0.93

        elif nearest_dist > SAFETY.danger_distance:
            # ── DANGER (10–15 m) ──────────────────────────────────────────
            self.status = VehicleStatus.DANGER
            self.target_speed = max(0.0, self.ego.vy * 0.25)
            if not self.is_changing_lane and do_lc:
                self._try_lane_change(all_objects)
            if prev_status != VehicleStatus.DANGER:
                self.braking_events += 1

        else:
            # ── EMERGENCY (< 10 m) ────────────────────────────────────────
            self.status = VehicleStatus.EMERGENCY
            self.target_speed = 0.0
            # Never lane-change for pedestrians/animals — only for static/stopped obstacles
            if not self.is_changing_lane and do_lc:
                self._try_lane_change(all_objects)
            if prev_status != VehicleStatus.EMERGENCY:
                self.braking_events += 1

        # TTC override — critical safety check
        if self.ttc < SAFETY.ttc_emergency and nearest_dist < SAFETY.warning_distance:
            self.status = VehicleStatus.EMERGENCY
            self.target_speed = 0.0
        elif self.ttc < SAFETY.ttc_danger and nearest_dist < SAFETY.caution_distance:
            self.status = VehicleStatus.DANGER
            self.target_speed = min(self.target_speed, self.ego.vy * 0.4)

        # ── BRAKING PID ───────────────────────────────────────────────
        # Turn the safe-gap distance error into a brake force (0..1).
        self.brake_intensity = self._compute_brake_pid(nearest_obj, nearest_dist)

        # Hard safety override: deterministic full braking in the emergency
        # range / on a critical time-to-collision — never trust the PID here.
        if (self.status == VehicleStatus.EMERGENCY
                or (self.ttc < SAFETY.ttc_emergency
                    and nearest_dist < SAFETY.warning_distance)):
            self.brake_pid.reset()
            self.brake_intensity = 1.0

        if self.ego.vy < 0.1 and self.brake_intensity > 0:
            self.status = VehicleStatus.STOPPED

        # After clearing danger, count safe time and return to center lane
        if self.status == VehicleStatus.SAFE:
            self.safe_timer += self.dt
            if (self.safe_timer > 3.5
                    and not self.is_changing_lane
                    and self.ego.lane != LanePosition.CENTER):
                self._return_to_center(all_objects)
        else:
            self.safe_timer = 0.0

        # Update decision text for dashboard display
        self.decision_text = self._get_decision_text()

    def _return_to_center(self, all_objects: List[Obstacle]):
        """Initiate a lane change back to center when road is clear."""
        center_score = self._evaluate_lane_safety(LanePosition.CENTER, all_objects)
        if center_score > 0.7:
            self._start_lane_change(LanePosition.CENTER)

    def _try_lane_change(self, all_objects: List[Obstacle]):
        if self.is_changing_lane:
            return

        current_lane = self.ego.lane
        candidates = []
        if current_lane.value > 0:
            candidates.append(LanePosition(current_lane.value - 1))
        if current_lane.value < ROAD.num_lanes - 1:
            candidates.append(LanePosition(current_lane.value + 1))

        best_lane = None
        best_score = -1

        for target_lane in candidates:
            score = self._evaluate_lane_safety(target_lane, all_objects)
            if score > best_score:
                best_score = score
                best_lane = target_lane

        if best_lane is not None and best_score > 0.5:
            self._start_lane_change(best_lane)

    def _evaluate_lane_safety(self, lane: LanePosition, all_objects: List[Obstacle]) -> float:
        lane_x = get_lane_center_x(lane)
        min_front_dist = 999.0
        min_rear_dist = 999.0

        for obj in all_objects:
            lateral_dist = abs(obj.x - lane_x)
            effective_lateral = max(0.0, lateral_dist - obj.width / 2.0)
            if effective_lateral > ROAD.lane_width * 0.7:
                continue

            rel_y = obj.y - self.ego.y
            if rel_y > 0:
                min_front_dist = min(min_front_dist, rel_y)
            else:
                min_rear_dist = min(min_rear_dist, abs(rel_y))

        if min_front_dist < 15.0 or min_rear_dist < 8.0:
            return 0.0

        front_score = min(1.0, min_front_dist / 60.0)
        rear_score  = min(1.0, min_rear_dist / 30.0)
        return (front_score * 0.7 + rear_score * 0.3)

    def _start_lane_change(self, target_lane: LanePosition):
        self.is_changing_lane = True
        self.lane_change_progress = 0.0
        self.lane_change_from = self.ego.lane
        self.lane_change_to = target_lane
        self.lane_change_start_x = self.ego.x
        self.lane_change_target_x = get_lane_center_x(target_lane)
        self.lane_changes += 1
        self.get_logger().info(
            f'Lane change: {self.lane_change_from.name} → {target_lane.name}'
        )

    def _update_lane_change(self):
        self.lane_change_progress += self.dt / PHYSICS.lane_change_duration

        if self.lane_change_progress >= 1.0:
            self.lane_change_progress = 1.0
            self.is_changing_lane = False
            self.ego.lane = self.lane_change_to
            self.ego.x = self.lane_change_target_x
            return

        t = self.lane_change_progress
        s = t * t * (3.0 - 2.0 * t)   # smooth-step interpolation
        self.ego.x = self.lane_change_start_x + s * (self.lane_change_target_x - self.lane_change_start_x)

    # ─── EGO PHYSICS ─────────────────────────────────────────

    def _update_ego_physics(self):
        """
        Longitudinal dynamics.

        Two layers control the speed:
          1. AEB emergency braking (brake_intensity) — the SAFETY override.
             When active it commands the brakes directly and the PID is
             held in reset so its integral term cannot wind up.
          2. PID speed controller — the NORMAL path. It tracks target_speed
             by turning the speed error into an acceleration command:
                 + command  ->  give gas (accelerate)
                 − command  ->  ease off / brake gently
        """
        self.speed_error = self.target_speed - self.ego.vy

        if self.brake_intensity > 0:
            # ── Safety override: AEB is braking ──────────────────────────
            self.speed_pid.reset()
            self.pid_output = 0.0
            decel = (PHYSICS.comfort_deceleration
                     + (PHYSICS.max_deceleration - PHYSICS.comfort_deceleration)
                     * self.brake_intensity)
            self.ego.vy = max(0, self.ego.vy - decel * self.dt)
        else:
            # ── Normal cruise/ACC: PID tracks the target speed ───────────
            accel_cmd = self.speed_pid.update(self.target_speed, self.ego.vy, self.dt)
            self.pid_output = accel_cmd

            if accel_cmd >= 0:
                # Throttle: accelerate toward the target
                accel = min(PHYSICS.max_acceleration, accel_cmd)
                self.ego.vy = min(PHYSICS.max_speed_kmh / 3.6,
                                  self.ego.vy + accel * self.dt)
            else:
                # Negative command: ease off / light braking
                decel = min(PHYSICS.comfort_deceleration, -accel_cmd)
                self.ego.vy = max(0, self.ego.vy - decel * self.dt)

        self.ego.y += self.ego.vy * self.dt

    # ─── PUBLISH STATE ───────────────────────────────────────

    def _publish_state(self):
        all_obs = []
        for tv in self.traffic_vehicles:
            d = tv.to_dict()
            d['is_traffic'] = True
            all_obs.append(d)
        for obs in self.obstacles:
            if obs.active:
                d = obs.to_dict()
                d['is_traffic'] = False
                all_obs.append(d)

        state = {
            'ego': {
                'x': round(self.ego.x, 2),
                'y': round(self.ego.y, 2),
                'vx': round(self.ego.vx, 2),
                'vy': round(self.ego.vy, 2),
                'speed_kmh': round(self.ego.vy * 3.6, 1),
                'lane': self.ego.lane.value,
                'lane_name': self.ego.lane.name,
            },
            'aeb': {
                'status': self.status.value,
                'brake_intensity': round(self.brake_intensity, 2),
                'nearest_distance': round(self.nearest_distance, 1),
                'nearest_type': self.nearest_type,
                'ttc': round(self.ttc, 2) if self.ttc < 100 else -1,
                'is_changing_lane': self.is_changing_lane,
                'lane_change_progress': round(self.lane_change_progress, 2) if self.is_changing_lane else 0,
                'lane_change_to': self.lane_change_to.name if self.is_changing_lane else '',
            },
            'pid': {
                'target_speed_kmh': round(self.target_speed * 3.6, 1),
                'error_kmh': round(self.speed_error * 3.6, 1),
                'output': round(self.pid_output, 2),
                'p': round(self.speed_pid.debug.p, 2),
                'i': round(self.speed_pid.debug.i, 2),
                'd': round(self.speed_pid.debug.d, 2),
            },
            'brake_pid': {
                'desired_gap': round(self.desired_gap, 1),
                'actual_gap': round(self.nearest_distance, 1) if self.nearest_distance < 500 else -1,
                'gap_error': round(self.gap_error, 1),
                'output': round(self.brake_intensity, 2),
                'p': round(self.brake_pid.debug.p, 2),
                'i': round(self.brake_pid.debug.i, 2),
                'd': round(self.brake_pid.debug.d, 2),
            },
            'scenario': {
                'active': self.active_scenario or '',
                'name': SCENARIO_NAMES.get(self.active_scenario, '') if self.active_scenario else '',
                'state': self.scenario_state,
                'timer': round(self.scenario_phase_timer, 1),
            },
            'decision_text': self.decision_text,
            'obstacles': all_obs,
            'lidar': self.lidar_points,
            'stats': {
                'total_distance': round(self.total_distance, 1),
                'braking_events': self.braking_events,
                'lane_changes': self.lane_changes,
                'uptime': round(time.time() - self.start_time, 1),
            }
        }

        msg = String()
        msg.data = json.dumps(state)
        self.pub_state.publish(msg)

        speed_msg = Float64()
        speed_msg.data = self.ego.vy * 3.6
        self.pub_speed.publish(speed_msg)

        lane_msg = Int32()
        lane_msg.data = self.ego.lane.value
        self.pub_lane.publish(lane_msg)


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    world_sim = WorldSimulation()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(world_sim)

    try:
        print("\n" + "=" * 60)
        print("  AUTONOMOUS EMERGENCY BRAKING SYSTEM")
        print("  ROS2 Humble | University Final Project")
        print("=" * 60)
        print("\n  Scenarios trigger every 3–6 seconds")
        print("  Press Ctrl+C to stop\n")
        executor.spin()
    except KeyboardInterrupt:
        print(f"\n  Stopped. Distance: {world_sim.total_distance:.1f}m")
        print(f"  Braking events: {world_sim.braking_events}")
        print(f"  Lane changes: {world_sim.lane_changes}")
    finally:
        executor.shutdown()
        world_sim.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
