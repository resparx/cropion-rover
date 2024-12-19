import cv2

class Camera:
    def __init__(self, camera_id: int = 0):
        """Initialize camera capture device
        
        Args:
            camera_id: ID of the camera device (default is 0 for primary camera)
        """
        self.camera_id = camera_id
        self.capture = None

    def start(self) -> None:
        """Start video capture"""
        self.capture = cv2.VideoCapture(self.camera_id)
        if not self.capture.isOpened():
            raise RuntimeError(f"Failed to open camera with ID {self.camera_id}")

    def get_frame(self) -> tuple[bool, cv2.typing.MatLike]:
        """Get the next frame from the video stream
        
        Returns:
            Tuple of (success, frame) where success is a boolean indicating if
            frame was successfully captured and frame is the image data
        """
        if self.capture is None:
            raise RuntimeError("Camera not started. Call start() first")
        return self.capture.read()

    def stop(self) -> None:
        """Stop video capture and release resources"""
        if self.capture is not None:
            self.capture.release()
            self.capture = None
