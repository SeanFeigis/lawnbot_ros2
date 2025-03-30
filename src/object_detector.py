import concurrent.futures
import rclpy
from ultralytics import YOLO
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_message.msg import BoundingBox
from std_srvs.srv import Trigger
import time

# def setup_model():
#     self.get_logger().info('Object Detector Node Started')
#     model = 
#     self.get_logger().info('Object Detector Node Started')
#     return model

MODEL_LOCATION="/home/lawnbot/ROS2/src/lawnbot_ros2/model/best_480_full_integer_quant_edgetpu.tflite"
MODEL_SIZE = 640

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            'image',  # Topic name must match the publisher's topic
            self.listener_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        self.bridge = CvBridge()
        #self.new_frame_available = False
        
        self.ready = False
        self.service = self.create_service(Trigger, 'model_ready', self.handle_ready_check)
        
        self.model = YOLO(MODEL_LOCATION, task='detect')
        self.test_readiness()  # Check if the model is ready
        
        ## Communicate to other nodes model is ready
        self.ready = True
        
        self.subscription  # Prevent unused variable warning
        
        
        self.publisher = self.create_publisher(
            BoundingBox,
            'bound_box',
            10
        )
        
        self.get_logger().info('Object Detector Node Started')
        
    def test_readiness(self):
        while True:
            dummy_image = np.zeros((640, 640, 3))
            self.model.predict(dummy_image, imgsz=480)
            self.get_logger().info("Model is ready for detection.")
            break
        
    def handle_ready_check(self, request, response):
        response.success = self.ready
        response.message = "Model is ready" if self.ready else "Model is not ready yet"
        return response   
        

    def listener_callback(self, msg):
        self.get_logger().debug('Received Image from PiCamera')
        try:
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            ###Uncomment for model rate testing
            #if not hasattr(self, 'last_prediction_time'):
                #self.last_prediction_time = time.time()
                
            results = self.model.predict(cv_image, imgsz=480)
            
            ###Uncomment for model rate testing
            #start_time = time.time()
            #elapsed_time = start_time - self.last_prediction_time
            #self.last_prediction_time = start_time
            #self.get_logger().info(f'Finished Predicting, time: {str(results[0].speed)}')
            #self.get_logger().info(f'Time since last prediction: {elapsed_time:.4f} seconds')
            
            for result in results:
                boxes = result.boxes.xywh  # Get the bounding boxes
                for box in boxes:
                    bbox = BoundingBox(center_x = box[0].item(), 
                                       center_y = box[1].item(),
                                       width    = box[2].item(),
                                       height   = box[3].item())
                    
                    #self.get_logger().info(f'Publishing: {str(bbox)}')
                    self.publisher.publish(bbox)
                    
                    
        except Exception as e:
            self.get_logger().error(f'Failed to predict model: {e}')

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()
    
    rclpy.spin(object_detector)
    
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
