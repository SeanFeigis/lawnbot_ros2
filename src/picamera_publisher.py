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
    config = picam2.create_still_configuration()
    #config = self.picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)    
    picam2.start()
    
    return picam2



class PiCameraPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_topic', 10)
        self.picam2 = setup_camera()
        self.timer = self.create_timer(1, self.timer_callback)
        
    def timer_callback(self):
        frame = self.picam2.capture_array()  # Capture image as a NumPy array
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        self.publisher_.publish(ros_image)
        self.get_logger().info('Taking Picture from PiCamera')

def main(args=None):
    rclpy.init(args=args)

    picamera_publisher = PiCameraPublisher()

    rclpy.spin(picamera_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    picamera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
