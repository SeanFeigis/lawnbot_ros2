import rclpy
from picamera2 import Picamera2
import numpy as np
from time import sleep

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.node import Node

from std_msgs.msg import String

def setup_camera():
    """Set up the camera to publish images for the model"""
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"format": "RGB888"})
    picam2.configure(config)    
    picam2.start()
    sleep(1) # Allow the camera to warm up
    
    return picam2



class PiCameraController(Node):

    def __init__(self):
        super().__init__('picamera_controller')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.picam2 = setup_camera()
        self.timer = self.create_timer(0.25, self.timer_callback)
        self.bridge = CvBridge()
        
    def timer_callback(self):
        frame = self.picam2.capture_array()  # Capture image as a NumPy array
        rotated_image = np.rot90(frame, 2)
        
        ros_image = self.bridge.cv2_to_imgmsg(rotated_image, encoding='rgb8')
        self.publisher_.publish(ros_image)
        self.get_logger().info('Taking Picture from PiCamera and Publishing')
        
    def shutdown_camera(self):
        """Shutdown the camera"""
        self.picam2.close()
        print("Camera shutdown")

def main(args=None):
    rclpy.init(args=args)

    picamera_controller = PiCameraController()

    rclpy.spin(picamera_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    picamera_controller.shutdown_camera()
    picamera_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
