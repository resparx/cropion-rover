#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import time
from navigation.path_planner import PathPlanner, GeoBoundary
from navigation.obstacle_avoidance import ObstacleAvoidance
from sensors.gps import GPSModule
from sensors.camera import CameraModule

class Rover(Node):
    def __init__(self):
        # Initialize ROS node
        rclpy.init()
        super().__init__('cropion_rover')
        
        # Initialize core systems
        self.gps = GPSModule()
        
        # Initialize navigation systems
        self.path_planner = PathPlanner(self.gps)
        self.obstacle_avoidance = ObstacleAvoidance()
        
        # ROS Publishers
        self.pose_pub = self.create_publisher(PoseStamped, 'rover/pose', 10)
        self.path_pub = self.create_publisher(Path, 'rover/path', 10)
        
        # ROS Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'rover/scan',
            self.laser_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz
        
        # Define initial operating boundary
        self.boundary = GeoBoundary(
            north_lat=37.7850,
            south_lat=37.7830,
            east_lon=-122.4000,
            west_lon=-122.4020
        )
        
        self.laser_scan_data = None
    
    def laser_callback(self, msg):
        """Callback for laser scan data"""
        self.laser_scan_data = msg.ranges
    
    def control_loop(self):
        """Main control loop for the rover"""
        try:
            # Get sensor readings
            current_pos = self.gps.get_position()
            
            if current_pos and self.laser_scan_data:
                # Process sensor data
                obstacles = self.obstacle_avoidance.detect_obstacles(self.laser_scan_data)
                
                # If we have a current path, check if it needs adjustment
                if hasattr(self, 'current_path'):
                    adjusted_path = self.obstacle_avoidance.adjust_path(
                        self.current_path, 
                        obstacles
                    )
                    self.current_path = self.obstacle_avoidance.smooth_path(adjusted_path)
                    
                    # Publish path
                    path_msg = self.create_path_message(self.current_path)
                    self.path_pub.publish(path_msg)
                
                # Publish current pose
                pose_msg = self.create_pose_message(current_pos)
                self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error in rover control loop: {e}")
    
    def create_pose_message(self, gps_pos):
        """Create PoseStamped message from GPS position"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = gps_pos["longitude"]
        msg.pose.position.y = gps_pos["latitude"]
        msg.pose.position.z = gps_pos["altitude"]
        return msg
    
    def create_path_message(self, path):
        """Create Path message from waypoints"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        for waypoint in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            msg.poses.append(pose)
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    rover = Rover()
    try:
        rclpy.spin(rover)
    except KeyboardInterrupt:
        pass
    finally:
        rover.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
