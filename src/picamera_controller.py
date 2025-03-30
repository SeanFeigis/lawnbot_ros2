import cv2
import rclpy
from picamera2 import Picamera2
import numpy as np
from time import sleep

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

FRAME_CAPTURE_RATE = 0.05  # 20 FPS

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
        self.publisher_ = self.create_publisher(Image, 'image', rclpy.qos.qos_profile_sensor_data)
        self.picam2 = setup_camera()
        
        self.bridge = CvBridge()
        
        #Wait for model to be ready before starting the camera
        self.cli = self.create_client(Trigger, 'model_ready')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Model service not available, waiting...")
            
        self.check_model_status()
        
        #Capture Frame Twenty times a second
        self.timer = self.create_timer(FRAME_CAPTURE_RATE, self.timer_callback)
        
    def check_model_status(self):
        req = Trigger.Request()
        while rclpy.ok():
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info("Model is ready. Proceeding with picamera node execution.")
                break
            self.get_logger().info("Model not ready, retrying...")
        
    def timer_callback(self):
        frame = self.picam2.capture_array()  # Capture image as a NumPy array
        
        #Resize the image to 1280x720 to reduce bandwidth
        frame = cv2.resize(frame, (1280, 720))
        
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        self.publisher_.publish(ros_image)
        self.get_logger().debug('Taking Picture from PiCamera and Publishing')
        
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
