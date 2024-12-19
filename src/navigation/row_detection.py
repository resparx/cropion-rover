import cv2
import numpy as np
from sensors.camera import Camera

class RowDetector:
    def __init__(self):
        """Initialize row detector with camera"""
        self.camera = Camera()
        self.camera.start()
        
        # Parameters for row detection
        self.min_line_length = 100  # Minimum length of line to detect
        self.max_line_gap = 50      # Maximum gap between line segments
        self.threshold = 50         # Minimum votes needed to detect line
        self.rho = 1               # Distance resolution in pixels
        self.theta = np.pi/180     # Angular resolution in radians
        
    def detect_rows(self):
        """Detect crop rows in camera frame
        
        Returns:
            List of detected row lines as (rho, theta) pairs
        """
        # Get frame from camera
        success, frame = self.camera.get_frame()
        if not success:
            return []
            
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(
            edges,
            self.rho,
            self.theta,
            self.threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap
        )
        
        # Filter lines to find parallel rows
        row_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
                
                # Only keep roughly vertical lines (adjust angles as needed)
                if 75 < abs(angle) < 105:
                    row_lines.append(line[0])
                    
        return row_lines
        
    def visualize_rows(self, frame, row_lines):
        """Draw detected rows on frame for visualization
        
        Args:
            frame: Original camera frame
            row_lines: List of detected row lines
        
        Returns:
            Frame with rows visualized
        """
        vis_frame = frame.copy()
        if row_lines:
            for x1, y1, x2, y2 in row_lines:
                cv2.line(vis_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return vis_frame
        
    def __del__(self):
        """Clean up camera resources"""
        self.camera.stop()
