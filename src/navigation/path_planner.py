import numpy as np
from typing import List, Tuple
from src.sensors.gps import GPSModule
from dataclasses import dataclass

@dataclass
class GeoBoundary:
    """Represents a geographical boundary with coordinates"""
    north_lat: float  # Northern latitude boundary
    south_lat: float  # Southern latitude boundary
    east_lon: float   # Eastern longitude boundary
    west_lon: float   # Western longitude boundary

    def contains_point(self, lat: float, lon: float) -> bool:
        """Check if a point lies within the boundary"""
        return (self.south_lat <= lat <= self.north_lat and 
                self.west_lon <= lon <= self.east_lon)

    def get_center(self) -> Tuple[float, float]:
        """Get the center point of the boundary"""
        center_lat = (self.north_lat + self.south_lat) / 2
        center_lon = (self.east_lon + self.west_lon) / 2
        return (center_lat, center_lon)

    def get_corners(self) -> List[Tuple[float, float]]:
        """Get the four corners of the boundary"""
        return [
            (self.north_lat, self.west_lon),  # Northwest
            (self.north_lat, self.east_lon),  # Northeast  
            (self.south_lat, self.east_lon),  # Southeast
            (self.south_lat, self.west_lon)   # Southwest
        ]

@dataclass 
class TraversalZone:
    """Defines an area for the rover to traverse"""
    boundary: GeoBoundary
    waypoints: List[Tuple[float, float]] = None
    
    def __post_init__(self):
        if self.waypoints is None:
            # Generate default waypoints in a lawn-mower pattern
            self.waypoints = self._generate_coverage_waypoints()
    
    def _generate_coverage_waypoints(self, spacing: float = 0.0001) -> List[Tuple[float, float]]:
        """Generate waypoints to cover the zone in a lawn-mower pattern"""
        points = []
        lat = self.boundary.south_lat
        going_east = True
        
        while lat <= self.boundary.north_lat:
            if going_east:
                points.append((lat, self.boundary.west_lon))
                points.append((lat, self.boundary.east_lon))
            else:
                points.append((lat, self.boundary.east_lon))
                points.append((lat, self.boundary.west_lon))
                
            lat += spacing
            going_east = not going_east
            
        return points

@dataclass
class PathPoint:
    latitude: float
    longitude: float
    heading: float

class PathPlanner:
    def __init__(self, gps: GPSModule, config_path: str = "config/path_planner_config.json"):
        self.gps = gps
        self.safety_margin = 0.5  # meters
        self.min_turn_radius = 1.0  # meters
        self.path_points: List[PathPoint] = []
        
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float], 
                  obstacles: List[Tuple[float, float, float]]) -> List[PathPoint]:
        """
        Plan a path from start to goal position avoiding obstacles
        
        Args:
            start: Tuple of (latitude, longitude) for start position
            goal: Tuple of (latitude, longitude) for goal position 
            obstacles: List of obstacle positions as (x, y, radius) tuples
            
        Returns:
            List of PathPoints representing the planned trajectory
        """
        # Convert lat/lon to local coordinates
        start_local = self._gps_to_local(start)
        goal_local = self._gps_to_local(goal)
        
        # Initialize RRT path planner
        path = self._rrt_plan(start_local, goal_local, obstacles)
        
        # Smooth path and add heading angles
        smooth_path = self._smooth_path(path)
        
        # Convert back to GPS coordinates
        self.path_points = []
        for point in smooth_path:
            lat, lon = self._local_to_gps(point[:2])
            self.path_points.append(PathPoint(
                latitude=lat,
                longitude=lon,
                heading=point[2]
            ))
            
        return self.path_points
    
    def _rrt_plan(self, start: np.ndarray, goal: np.ndarray, 
                  obstacles: List[Tuple[float, float, float]]) -> List[np.ndarray]:
        """Rapidly-exploring Random Tree (RRT) path planning"""
        nodes = [start]
        parents = [0]
        
        for _ in range(1000):  # Max iterations
            # Sample random point
            if np.random.random() < 0.1:
                sample = goal
            else:
                sample = np.random.uniform(low=-50, high=50, size=2)
                
            # Find nearest node
            distances = [np.linalg.norm(node - sample) for node in nodes]
            nearest_idx = np.argmin(distances)
            nearest = nodes[nearest_idx]
            
            # Extend towards sample
            direction = sample - nearest
            distance = np.linalg.norm(direction)
            if distance > self.min_turn_radius:
                direction = direction / distance * self.min_turn_radius
            new_node = nearest + direction
            
            # Check if path is collision free
            if not self._check_collision(nearest, new_node, obstacles):
                nodes.append(new_node)
                parents.append(nearest_idx)
                
                # Check if goal is reached
                if np.linalg.norm(new_node - goal) < self.min_turn_radius:
                    break
                    
        # Extract path by backtracking
        path = []
        current_idx = len(nodes) - 1
        while current_idx != 0:
            path.append(nodes[current_idx])
            current_idx = parents[current_idx]
        path.append(start)
        
        return path[::-1]
    
    def _smooth_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Apply path smoothing and add heading angles"""
        smooth = []
        for i in range(len(path)):
            if i == 0:
                heading = np.arctan2(path[1][1] - path[0][1],
                                   path[1][0] - path[0][0])
            elif i == len(path) - 1:
                heading = np.arctan2(path[-1][1] - path[-2][1],
                                   path[-1][0] - path[-2][0])
            else:
                heading = np.arctan2(path[i+1][1] - path[i-1][1],
                                   path[i+1][0] - path[i-1][0])
            smooth.append(np.array([path[i][0], path[i][1], heading]))
        return smooth
    
    def _check_collision(self, p1: np.ndarray, p2: np.ndarray, 
                        obstacles: List[Tuple[float, float, float]]) -> bool:
        """Check if path between two points collides with obstacles"""
        for ox, oy, radius in obstacles:
            obstacle = np.array([ox, oy])
            d = np.linalg.norm(np.cross(p2-p1, p1-obstacle))/np.linalg.norm(p2-p1)
            if d < (radius + self.safety_margin):
                return True
        return False
        
    def _gps_to_local(self, gps_coord: Tuple[float, float]) -> np.ndarray:
        """Convert GPS coordinates to local coordinate system"""
        # Use Haversine formula for conversion
        lat, lon = gps_coord
        R = 6371000  # Earth radius in meters
        
        ref_lat = self.gps.kalman_state[0]
        ref_lon = self.gps.kalman_state[1]
        
        x = R * np.cos(np.radians(lat)) * np.cos(np.radians(lon - ref_lon))
        y = R * np.cos(np.radians(lat)) * np.sin(np.radians(lon - ref_lon))
        
        return np.array([x, y])
    
    def _local_to_gps(self, local_coord: np.ndarray) -> Tuple[float, float]:
        """Convert local coordinates back to GPS coordinates"""
        x, y = local_coord
        R = 6371000
        
        ref_lat = self.gps.kalman_state[0]
        ref_lon = self.gps.kalman_state[1]
        
        lon = ref_lon + np.degrees(np.arctan2(y, x))
        lat = ref_lat + np.degrees(np.arcsin(np.sqrt(x*x + y*y) / R))
        
        return (lat, lon)
