import cv2
import rclpy
from picamera2 import Picamera2
import numpy as np
from time import sleep

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_srvs.srv import Trigger
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition

FRAME_CAPTURE_RATE = 0.05  # 20 FPS

def setup_camera():
    """Set up the camera to publish images for the model"""
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"format": "RGB888"})
    picam2.configure(config)    
    picam2.start()
    sleep(1)  # Allow the camera to warm up
    return picam2

class PiCameraController(Node):
    
    def __init__(self):
        super().__init__('picamera_controller')
        
        self._state = State.UNKNOWN
        self.publisher_ = self.create_publisher(Image, 'image', rclpy.qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        
        # Create timer but don't start it yet
        self.timer = None

    def configure(self):
        """Configure: Initialize camera when the node is configured."""
        self.get_logger().info("Configuring PiCamera...")
        
        # Initialize camera setup here
        self.picam2 = setup_camera()
        
        return Transition.TRANSITION_CONFIGURE

    def activate(self):
        """Activate: Start the timer to capture and publish images."""
        self.get_logger().info("Activating PiCamera, starting image capture timer.")
        
        # Start the timer to capture frames
        self.timer = self.create_timer(FRAME_CAPTURE_RATE, self.timer_callback)
        
        return Transition.TRANSITION_ACTIVATE

    def deactivate(self):
        """Deactivate: Stop the timer to stop capturing images."""
        self.get_logger().info("Deactivating PiCamera, stopping image capture timer.")
        
        # Stop the timer (image capture)
        if self.timer:
            self.timer.cancel()
            self.timer = None
        
        return Transition.TRANSITION_DEACTIVATE

    def cleanup(self):
        """Cleanup: Shutdown camera."""
        self.get_logger().info("Cleaning up PiCamera, shutting down camera.")
        
        # Shutdown the camera
        self.shutdown_camera()
        
        return Transition.TRANSITION_CLEANUP

    def shutdown_camera(self):
        """Shutdown the camera."""
        if hasattr(self, 'picam2'):
            self.picam2.close()
            self.get_logger().info("Camera shutdown successfully.")

    def timer_callback(self):
        """Capture a frame and publish it."""
        frame = self.picam2.capture_array()  # Capture image as a NumPy array
        
        # Resize the image to 1280x720 to reduce bandwidth
        frame = cv2.resize(frame, (1280, 720))
        
        # Convert to ROS2 Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        self.publisher_.publish(ros_image)
        self.get_logger().debug('Taking Picture from PiCamera and Publishing')

def main(args=None):
    rclpy.init(args=args)

    picamera_controller = PiCameraController()

    rclpy.spin(picamera_controller)

    picamera_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
