"""
═══════════════════════════════════════════════════════════════════════════════
    LIDAR SENSOR SIMULATION MODULE
    Simulates a multi-layer LIDAR sensor with realistic characteristics
═══════════════════════════════════════════════════════════════════════════════
"""

import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import numpy as np

from .config import LIDAR, ROAD, LanePosition
from .obstacle_manager import Obstacle


@dataclass
class LidarPoint:
    """Single LIDAR point"""
    x: float          # meters
    y: float          # meters
    z: float          # meters
    intensity: float  # 0.0 to 1.0
    distance: float   # meters
    angle_h: float    # horizontal angle (radians)
    angle_v: float    # vertical angle (radians)
    layer: int        # LIDAR layer/ring number
    
    def to_tuple(self) -> Tuple[float, float, float, float]:
        return (self.x, self.y, self.z, self.intensity)


@dataclass
class LidarScan:
    """Complete LIDAR scan data"""
    timestamp: float
    points: List[LidarPoint] = field(default_factory=list)
    num_points: int = 0
    
    # Detected objects
    detected_obstacles: List[dict] = field(default_factory=list)


@dataclass
class DetectedObject:
    """Object detected by LIDAR processing"""
    center_x: float
    center_y: float
    center_z: float
    width: float
    length: float
    height: float
    distance: float
    velocity_estimate: float
    confidence: float
    classification: str
    point_count: int


class LidarSensor:
    """
    Simulated multi-layer LIDAR sensor
    
    Features:
    - Multi-layer vertical scanning
    - Configurable angular resolution
    - Realistic noise model
    - Ground plane detection
    - Object clustering
    """
    
    def __init__(self):
        self.config = LIDAR
        
        # Pre-calculate scan angles
        self.h_angles = self._generate_horizontal_angles()
        self.v_angles = self._generate_vertical_angles()
        
        # Previous scan for velocity estimation
        self.previous_scan: Optional[LidarScan] = None
        self.previous_detections: Dict[int, DetectedObject] = {}
        
        # Ground plane parameters
        self.ground_height = -0.5  # meters below sensor
        
    def _generate_horizontal_angles(self) -> np.ndarray:
        """Generate horizontal scan angles"""
        num_points = int(self.config.horizontal_fov / self.config.angular_resolution)
        start = -math.radians(self.config.horizontal_fov / 2)
        end = math.radians(self.config.horizontal_fov / 2)
        return np.linspace(start, end, num_points)
    
    def _generate_vertical_angles(self) -> np.ndarray:
        """Generate vertical scan angles for each layer"""
        start = -math.radians(self.config.vertical_fov / 2)
        end = math.radians(self.config.vertical_fov / 2)
        return np.linspace(start, end, self.config.num_layers)
    
    def scan(self, ego_x: float, ego_y: float, ego_z: float,
             obstacles: List[Obstacle], timestamp: float) -> LidarScan:
        """
        Perform a LIDAR scan
        
        Args:
            ego_x, ego_y, ego_z: Sensor position
            obstacles: List of obstacles in the scene
            timestamp: Current simulation time
            
        Returns:
            LidarScan containing all detected points
        """
        scan = LidarScan(timestamp=timestamp)
        points = []
        
        # Scan each layer
        for layer, v_angle in enumerate(self.v_angles):
            # Scan horizontally
            for h_angle in self.h_angles:
                point = self._cast_ray(
                    ego_x, ego_y, ego_z,
                    h_angle, v_angle, layer,
                    obstacles
                )
                if point is not None:
                    points.append(point)
        
        scan.points = points
        scan.num_points = len(points)
        
        # Process scan to detect objects
        scan.detected_obstacles = self._process_scan(scan, obstacles)
        
        # Store for next frame
        self.previous_scan = scan
        
        return scan
    
    def _cast_ray(self, ego_x: float, ego_y: float, ego_z: float,
                  h_angle: float, v_angle: float, layer: int,
                  obstacles: List[Obstacle]) -> Optional[LidarPoint]:
        """Cast a single ray and find intersection"""
        
        # Ray direction
        cos_v = math.cos(v_angle)
        dir_x = math.sin(h_angle) * cos_v
        dir_y = math.cos(h_angle) * cos_v
        dir_z = math.sin(v_angle)
        
        min_distance = self.config.max_range
        hit_obstacle = None
        
        # Check ground plane intersection (only for downward rays)
        if dir_z < -0.01:
            t_ground = (self.ground_height - ego_z) / dir_z
            if self.config.min_range < t_ground < min_distance:
                # Ground hit - add some noise for road texture
                ground_distance = t_ground + random.gauss(0, 0.05)
                if ground_distance < min_distance:
                    min_distance = ground_distance
        
        # Check obstacle intersections
        for obstacle in obstacles:
            if not obstacle.is_active:
                continue
            
            # Simple AABB intersection test
            distance = self._ray_box_intersection(
                ego_x, ego_y, ego_z,
                dir_x, dir_y, dir_z,
                obstacle
            )
            
            if distance is not None and distance < min_distance:
                min_distance = distance
                hit_obstacle = obstacle
        
        # Check if within valid range
        if min_distance < self.config.min_range or min_distance >= self.config.max_range:
            return None
        
        # Add measurement noise
        noisy_distance = min_distance + random.gauss(0, self.config.noise_std)
        
        # Calculate point position
        px = ego_x + dir_x * noisy_distance
        py = ego_y + dir_y * noisy_distance
        pz = ego_z + dir_z * noisy_distance
        
        # Calculate intensity (varies with distance and material)
        base_intensity = 1.0 - (noisy_distance / self.config.max_range)
        if hit_obstacle:
            # Different materials have different reflectivity
            intensity = base_intensity * self._get_reflectivity(hit_obstacle.obstacle_type.value)
        else:
            intensity = base_intensity * 0.3  # Ground
        
        return LidarPoint(
            x=px, y=py, z=pz,
            intensity=max(0.0, min(1.0, intensity)),
            distance=noisy_distance,
            angle_h=h_angle,
            angle_v=v_angle,
            layer=layer
        )
    
    def _ray_box_intersection(self, ox: float, oy: float, oz: float,
                               dx: float, dy: float, dz: float,
                               obstacle: Obstacle) -> Optional[float]:
        """
        Ray-AABB intersection test
        
        Returns distance to intersection or None
        """
        # Obstacle bounds
        min_x = obstacle.x - obstacle.width / 2
        max_x = obstacle.x + obstacle.width / 2
        min_y = obstacle.y - obstacle.length / 2
        max_y = obstacle.y + obstacle.length / 2
        min_z = 0.0
        max_z = obstacle.height
        
        t_min = 0.0
        t_max = float('inf')
        
        # X axis
        if abs(dx) > 1e-8:
            t1 = (min_x - ox) / dx
            t2 = (max_x - ox) / dx
            t_min = max(t_min, min(t1, t2))
            t_max = min(t_max, max(t1, t2))
        elif ox < min_x or ox > max_x:
            return None
        
        # Y axis
        if abs(dy) > 1e-8:
            t1 = (min_y - oy) / dy
            t2 = (max_y - oy) / dy
            t_min = max(t_min, min(t1, t2))
            t_max = min(t_max, max(t1, t2))
        elif oy < min_y or oy > max_y:
            return None
        
        # Z axis
        if abs(dz) > 1e-8:
            t1 = (min_z - oz) / dz
            t2 = (max_z - oz) / dz
            t_min = max(t_min, min(t1, t2))
            t_max = min(t_max, max(t1, t2))
        elif oz < min_z or oz > max_z:
            return None
        
        if t_max >= t_min and t_min > 0:
            return t_min
        return None
    
    def _get_reflectivity(self, obstacle_type: str) -> float:
        """Get material reflectivity for different obstacle types"""
        reflectivity = {
            'car': 0.9,
            'pedestrian': 0.5,
            'child': 0.45,
            'ball': 0.7,
            'fallen_tree': 0.35,
            'motorcycle': 0.85,
            'bicycle': 0.6,
            'animal': 0.4,
        }
        return reflectivity.get(obstacle_type, 0.5)
    
    def _process_scan(self, scan: LidarScan, 
                      known_obstacles: List[Obstacle]) -> List[dict]:
        """
        Process LIDAR scan to detect and classify objects
        
        This is a simplified version - in real systems this would use
        clustering algorithms like DBSCAN and ML classifiers
        """
        detections = []
        
        for obstacle in known_obstacles:
            if not obstacle.is_active:
                continue
            
            # Count points that hit this obstacle
            point_count = 0
            min_distance = float('inf')
            
            for point in scan.points:
                # Check if point is on this obstacle
                if (abs(point.x - obstacle.x) <= obstacle.width / 2 + 0.2 and
                    abs(point.y - obstacle.y) <= obstacle.length / 2 + 0.2 and
                    0 <= point.z <= obstacle.height + 0.2):
                    point_count += 1
                    min_distance = min(min_distance, point.distance)
            
            if point_count > 3:  # Minimum points for detection
                detection = {
                    'id': obstacle.id,
                    'type': obstacle.obstacle_type.value,
                    'x': obstacle.x,
                    'y': obstacle.y,
                    'distance': min_distance,
                    'width': obstacle.width,
                    'length': obstacle.length,
                    'height': obstacle.height,
                    'velocity_x': obstacle.vx,
                    'velocity_y': obstacle.vy,
                    'confidence': min(1.0, point_count / 50.0),
                    'point_count': point_count,
                    'lane': obstacle.lane.value,
                }
                detections.append(detection)
        
        # Sort by distance
        detections.sort(key=lambda d: d['distance'])
        
        return detections
    
    def get_visualization_points(self, scan: LidarScan, 
                                 ego_x: float, ego_y: float) -> List[Tuple[float, float, float, Tuple[int, int, int]]]:
        """
        Get points for 2D/3D visualization
        
        Returns list of (x, y, distance, color) tuples
        """
        viz_points = []
        
        for point in scan.points:
            # Color based on distance
            dist_ratio = point.distance / self.config.max_range
            
            if dist_ratio < 0.2:
                color = (255, 50, 50)  # Red - close
            elif dist_ratio < 0.4:
                color = (255, 165, 0)  # Orange
            elif dist_ratio < 0.6:
                color = (255, 255, 0)  # Yellow
            elif dist_ratio < 0.8:
                color = (100, 255, 100)  # Green
            else:
                color = (100, 100, 255)  # Blue - far
            
            # Relative position
            rel_x = point.x - ego_x
            rel_y = point.y - ego_y
            
            viz_points.append((rel_x, rel_y, point.distance, color))
        
        return viz_points
    
    def get_scan_statistics(self, scan: LidarScan) -> dict:
        """Get statistical information about a scan"""
        if not scan.points:
            return {
                'num_points': 0,
                'min_distance': 0,
                'max_distance': 0,
                'avg_distance': 0,
                'num_detections': 0,
            }
        
        distances = [p.distance for p in scan.points]
        
        return {
            'num_points': scan.num_points,
            'min_distance': min(distances),
            'max_distance': max(distances),
            'avg_distance': sum(distances) / len(distances),
            'num_detections': len(scan.detected_obstacles),
        }
