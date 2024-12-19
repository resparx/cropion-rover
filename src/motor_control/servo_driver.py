import serial
import time

class ServoDriver:
    def __init__(self, port='/dev/ttyUSB1', baud_rate=9600):
        """Initialize servo driver with Arduino serial connection
        
        Args:
            port: Serial port Arduino is connected to
            baud_rate: Baud rate for serial communication
        """
        self.serial = serial.Serial(port, baud_rate)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Servo angle limits (degrees)
        self.min_angle = 0
        self.max_angle = 180
        self.current_angle = 90  # Start at center position
        
    def set_angle(self, angle: int) -> None:
        """Set servo angle
        
        Args:
            angle: Desired angle in degrees (0-180)
        """
        # Constrain angle to valid range
        angle = max(min(angle, self.max_angle), self.min_angle)
        self.current_angle = angle
        
        # Send command to Arduino
        command = f"S,{angle}\n"
        self.serial.write(command.encode())
        
    def center(self) -> None:
        """Move servo to center position (90 degrees)"""
        self.set_angle(90)
        
    def __del__(self):
        """Clean up serial connection"""
        if hasattr(self, 'serial'):
            self.center()  # Return to center position
            self.serial.close()
