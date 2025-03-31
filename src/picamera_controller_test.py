import cv2
import rclpy
from picamera2 import Picamera2
import numpy as np
import time

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.node import Node
from ultralytics import YOLO
from std_msgs.msg import String
from std_srvs.srv import Trigger
from std_msgs.msg import ByteMultiArray
from custom_message.msg import BoundingBox

MODEL_LOCATION="/home/lawnbot/ROS2/src/lawnbot_ros2/model/best_480_full_integer_quant_edgetpu.tflite"

def setup_camera():
    """Set up the camera to publish images for the model"""
    picam2 = Picamera2()
    config = picam2.create_still_configuration(main={"format": "RGB888"})
    picam2.configure(config)    
    picam2.start()
    #sleep(1) # Allow the camera to warm up
    
    return picam2



class PiCameraController(Node):

    def __init__(self):
        super().__init__('picamera_controller_test')
        self.publisher_ = self.create_publisher(Image, 'image', rclpy.qos.qos_profile_sensor_data)
        self.picam2 = setup_camera()
        
        self.bridge = CvBridge()
        
        # self.cli = self.create_client(Trigger, 'model_ready')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info("Model service not available, waiting...")
            
        # self.check_model_status()
        self.model = YOLO(MODEL_LOCATION, task='detect')
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        
    # def check_model_status(self):
    #     req = Trigger.Request()
    #     while rclpy.ok():
    #         future = self.cli.call_async(req)
    #         rclpy.spin_until_future_complete(self, future)
    #         if future.result().success:
    #             self.get_logger().info("Model is ready. Proceeding with node execution.")
    #             break
    #         self.get_logger().info("Model not ready, retrying...")
        
    def timer_callback(self):
        frame = self.picam2.capture_array()  # Capture image as a NumPy array
        #frame = cv2.resize(frame, (640, 640))
        #rotated_image = np.rot90(frame, 2)
        
        
        
        # ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        # frame = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")
        # #self.publisher_.publish(ros_image)
        # #self.get_logger().debug('Taking Picture from PiCamera and Publishing')
        
        # img_bytes = frame.tobytes()
        # msg = ByteMultiArray()
        # msg.data = list(img_bytes)
        
        # np_image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape((640, 640, 3))
        
        
        if not hasattr(self, 'last_prediction_time'):
            self.last_prediction_time = time.time()
        
        
        results = self.model.predict(frame, imgsz=480)
        start_time = time.time()
        
        elapsed_time = start_time - self.last_prediction_time
        self.last_prediction_time = start_time
        
        self.get_logger().info(f'Finished Predicting, time: {str(results[0].speed)}')
        self.get_logger().info(f'Time since last prediction: {elapsed_time:.4f} seconds')
        #self.get_logger().info(f'Finished Predicting, inference time: {results[0].speed}')
        
        # Process the results and publish the bounding boxes
        for result in results:
            boxes = result.boxes.xywh  # Get the bounding boxes
            for box in boxes:
                bbox = BoundingBox(center_x=box[0].item(), 
                                   center_y=box[1].item(),
                                   width=box[2].item(),
                                   height=box[3].item())
                #self.get_logger().info(f'Publishing: {str(bbox)}')
                #self.publisher.publish(bbox)
        
        
        
    def shutdown_camera(self):
        """Shutdown the camera"""
        self.picam2.close()
        print("Camera shutdown")

def main(args=None):
    rclpy.init(args=args)

    picamera_controller = PiCameraController()
    try:
        rclpy.spin(picamera_controller)
    except KeyboardInterrupt:
        picamera_controller.shutdown_camera()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    picamera_controller.shutdown_camera()
    picamera_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
