import time
from navigation.path_planner import PathPlanner, GeoBoundary
from navigation.obstacle_avoidance import ObstacleAvoidance
from sensors.gps import GPSModule

class Rover:
    def __init__(self):
        # Initialize core systems
        self.gps = GPSModule()  # Assuming GPSModule is imported/defined elsewhere
        
        # Initialize navigation systems
        self.path_planner = PathPlanner(self.gps)
        self.obstacle_avoidance = ObstacleAvoidance()
        
        # Define initial operating boundary
        self.boundary = GeoBoundary(
            north_lat=37.7850,  # Example coordinates
            south_lat=37.7830,
            east_lon=-122.4000,
            west_lon=-122.4020
        )
        
    def run(self):
        """Main control loop for the rover"""
        while True:
            try:
                # Get sensor readings
                current_pos = self.gps.get_position()
                laser_scan = self.lidar.get_scan()
                
                # Process sensor data
                obstacles = self.obstacle_avoidance.detect_obstacles(laser_scan)
                
                # If we have a current path, check if it needs adjustment
                if hasattr(self, 'current_path'):
                    adjusted_path = self.obstacle_avoidance.adjust_path(
                        self.current_path, 
                        obstacles
                    )
                    self.current_path = self.obstacle_avoidance.smooth_path(adjusted_path)
                
                time.sleep(0.1)  # Control loop rate
                
            except Exception as e:
                print(f"Error in rover control loop: {e}")
                time.sleep(1)  # Brief pause before retrying

if __name__ == "__main__":
    rover = Rover()
    rover.run()
