import math

class ObstacleAvoidance:
    def __init__(self):
        self.safety_margin = 0.5  # meters
        self.scan_range = 180  # degrees
        self.min_distance = 1.0  # minimum distance to obstacle

    def detect_obstacles(self, laser_scan_data):
        """
        Process laser scan data to detect obstacles
        Returns list of detected obstacles with their positions
        """
        obstacles = []
        for angle, distance in enumerate(laser_scan_data):
            if distance < self.min_distance:
                # Convert polar coordinates to Cartesian
                x = distance * math.cos(math.radians(angle))
                y = distance * math.sin(math.radians(angle))
                obstacles.append((x, y))
        return obstacles

    def adjust_path(self, current_path, obstacles):
        """
        Modify path to avoid detected obstacles
        Returns adjusted path
        """
        adjusted_path = current_path.copy()
        
        for obstacle in obstacles:
            # Check if obstacle interferes with path
            for i, waypoint in enumerate(adjusted_path):
                distance = math.sqrt((waypoint[0] - obstacle[0])**2 + 
                                   (waypoint[1] - obstacle[1])**2)
                
                if distance < self.safety_margin:
                    # Calculate avoidance vector perpendicular to path
                    dx = waypoint[0] - obstacle[0]
                    dy = waypoint[1] - obstacle[1]
                    magnitude = math.sqrt(dx**2 + dy**2)
                    
                    # Normalize and scale by safety margin
                    if magnitude > 0:
                        dx = (dx / magnitude) * self.safety_margin
                        dy = (dy / magnitude) * self.safety_margin
                        
                        # Update waypoint position
                        adjusted_path[i] = (waypoint[0] + dx, waypoint[1] + dy)
        
        return adjusted_path

    def smooth_path(self, path):
        """
        Apply smoothing to avoid sharp turns in adjusted path
        Returns smoothed path
        """
        smoothed_path = []
        if len(path) < 3:
            return path
            
        for i in range(1, len(path)-1):
            # Average position with neighbors
            x = (path[i-1][0] + path[i][0] + path[i+1][0]) / 3
            y = (path[i-1][1] + path[i][1] + path[i+1][1]) / 3
            smoothed_path.append((x, y))
            
        # Keep start and end points unchanged
        smoothed_path.insert(0, path[0])
        smoothed_path.append(path[-1])
        
        return smoothed_path
