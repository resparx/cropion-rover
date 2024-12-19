import serial
import time

class MotorDriver:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600):
        """Initialize motor driver with Arduino serial connection
        
        Args:
            port: Serial port Arduino is connected to
            baud_rate: Baud rate for serial communication
        """
        self.serial = serial.Serial(port, baud_rate)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Motor speed limits (0-255)
        self.min_speed = 0
        self.max_speed = 255
        
        # Current motor states
        self.front_speed = 0
        self.front_direction = 1  # 1 forward, -1 reverse
        self.rear_speed = 0 
        self.rear_direction = 1
        
    def set_front_motor(self, speed: int, direction: int = 1) -> None:
        """Set front axle motor speed and direction
        
        Args:
            speed: Motor speed (0-255)
            direction: 1 for forward, -1 for reverse
        """
        speed = max(min(abs(speed), self.max_speed), self.min_speed)
        direction = 1 if direction >= 0 else -1
        
        self.front_speed = speed
        self.front_direction = direction
        
        # Send command to Arduino
        command = f"F,{speed},{direction}\n"
        self.serial.write(command.encode())
        
    def set_rear_motor(self, speed: int, direction: int = 1) -> None:
        """Set rear axle motor speed and direction
        
        Args:
            speed: Motor speed (0-255) 
            direction: 1 for forward, -1 for reverse
        """
        speed = max(min(abs(speed), self.max_speed), self.min_speed)
        direction = 1 if direction >= 0 else -1
        
        self.rear_speed = speed
        self.rear_direction = direction
        
        # Send command to Arduino
        command = f"R,{speed},{direction}\n"
        self.serial.write(command.encode())
        
    def stop_all(self) -> None:
        """Stop all motors"""
        self.set_front_motor(0)
        self.set_rear_motor(0)
        
    def __del__(self):
        """Clean up serial connection"""
        if hasattr(self, 'serial'):
            self.stop_all()
            self.serial.close()
