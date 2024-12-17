import json
import serial
import time
from adafruit_gps import GPS
import numpy as np
from pathlib import Path

class GPSModule:
    def __init__(self, config_path="config/gps_config.json"):
        # Load configuration
        self.config = self._load_config(config_path)
        
        # Initialize serial connection
        self.uart = serial.Serial(
            port=self.config["gps_settings"]["port"],
            baudrate=self.config["gps_settings"]["baudrate"],
            timeout=self.config["gps_settings"]["timeout"]
        )
        
        # Create GPS object
        self.gps = GPS(self.uart, debug=False)
        
        # Initialize Kalman filter variables if enabled
        if self.config["filtering"]["enable_kalman_filter"]:
            self.pos_variance = self.config["filtering"]["position_variance"]
            self.vel_variance = self.config["filtering"]["velocity_variance"]
            self.measurement_variance = self.config["filtering"]["measurement_variance"]
            self.kalman_state = np.zeros(4)  # [lat, lon, lat_vel, lon_vel]
            self.kalman_covariance = np.eye(4)
    
    def _load_config(self, config_path):
        """Load GPS configuration from JSON file"""
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except Exception as e:
            raise Exception(f"Failed to load GPS config: {str(e)}")
    
    def _apply_kalman_filter(self, lat, lon):
        """Apply Kalman filtering to GPS coordinates"""
        # Prediction step
        dt = 1.0 / self.config["gps_settings"]["update_frequency"]
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        self.kalman_state = F @ self.kalman_state
        self.kalman_covariance = F @ self.kalman_covariance @ F.T + \
                                np.eye(4) * self.vel_variance
        
        # Update step
        measurement = np.array([lat, lon, 0, 0])
        H = np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 0, 0],
                     [0, 0, 0, 0]])
        
        R = np.eye(4) * self.measurement_variance
        S = H @ self.kalman_covariance @ H.T + R
        K = self.kalman_covariance @ H.T @ np.linalg.inv(S)
        
        self.kalman_state = self.kalman_state + K @ (measurement - H @ self.kalman_state)
        self.kalman_covariance = (np.eye(4) - K @ H) @ self.kalman_covariance
        
        return self.kalman_state[0], self.kalman_state[1]
    
    def get_position(self):
        """Get current GPS position with filtering and validation"""
        if not self.gps.update():
            return None
            
        # Check if we have a GPS fix
        if not self.gps.has_fix:
            return None
            
        # Check minimum satellites requirement
        if self.gps.satellites is not None and \
           self.gps.satellites < self.config["gps_settings"]["minimum_satellites"]:
            return None
            
        lat = self.gps.latitude
        lon = self.gps.longitude
        alt = self.gps.altitude_m if self.gps.altitude_m is not None else 0.0
        
        # Apply Kalman filtering if enabled
        if self.config["filtering"]["enable_kalman_filter"]:
            lat, lon = self._apply_kalman_filter(lat, lon)
        
        # Apply configured offsets
        lat += self.config["position_settings"]["latitude_offset"]
        lon += self.config["position_settings"]["longitude_offset"]
        alt += self.config["position_settings"]["altitude_offset"]
        
        return {
            "latitude": lat,
            "longitude": lon,
            "altitude": alt,
            "satellites": self.gps.satellites,
            "timestamp": time.time()
        }
