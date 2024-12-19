import math
from .motor_driver import MotorDriver
from .servo_driver import ServoDriver

class MovementController:
    def __init__(self, motor_driver: MotorDriver, servo_driver: ServoDriver):
        """Initialize movement controller
        
        Args:
            motor_driver: Motor driver for wheel control
            servo_driver: Servo driver for steering control
        """
        self.motor = motor_driver
        self.servo = servo_driver
        
        # Movement parameters
        self.max_speed = 200  # Conservative max speed
        self.min_turn_radius = 2.0  # meters
        self.wheel_base = 0.5  # meters between front and rear axles
        
    def move_to_coordinate(self, current_pos, target_pos):
        """Move rover from current position to target position
        
        Args:
            current_pos: Tuple of (x,y) current coordinates
            target_pos: Tuple of (x,y) target coordinates
        """
        # Calculate heading angle to target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        target_heading = math.degrees(math.atan2(dy, dx))
        
        # Convert heading to servo angle (90 is straight)
        # Constrain steering angle to ±45 degrees
        steering_angle = 90 + max(min(target_heading, 45), -45)
        self.servo.set_angle(int(steering_angle))
        
        # Calculate distance to target
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Set motor speed based on distance and turn angle
        # Reduce speed for sharp turns
        turn_factor = abs(90 - steering_angle) / 45.0  # 0 to 1
        speed = int(self.max_speed * (1 - 0.7 * turn_factor))
        
        # Drive motors
        self.motor.set_front_motor(speed, 1)
        self.motor.set_rear_motor(speed, 1)
        
    def stop(self):
        """Stop movement and center steering"""
        self.motor.stop_all()
        self.servo.center()
