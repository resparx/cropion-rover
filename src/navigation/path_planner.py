class GeoBoundary:
    def __init__(self, north_lat, south_lat, east_lon, west_lon):
        self.north_lat = north_lat
        self.south_lat = south_lat 
        self.east_lon = east_lon
        self.west_lon = west_lon

class PathPlanner:
    def __init__(self, gps_module):
        self.gps = gps_module
        self.min_turn_radius = 2.0  # meters
        self.traversed_area = []
        self.current_position = None
        
    def update_position(self):
        """Update current position from GPS and add to traversed area"""
        position = self.gps.get_position()
        self.current_position = position
        self.traversed_area.append(position)
        
    def set_boundary(self, boundary: GeoBoundary):
        """Set the operating boundary for the rover"""
        self.boundary = boundary
        
    def is_within_boundary(self, position) -> bool:
        """Check if position is within set boundary"""
        lat, lon = position
        return (self.boundary.south_lat <= lat <= self.boundary.north_lat and
                self.boundary.west_lon <= lon <= self.boundary.east_lon)
                
    def move_to_start(self, start_position):
        """Move rover to starting position"""
        if not self.is_within_boundary(start_position):
            raise ValueError("Start position outside boundary")
            
        self.update_position()
        # Basic straight line path to start
        while self.current_position != start_position:
            # Update position continuously while moving
            self.update_position()
            # Movement control would go here
            pass
