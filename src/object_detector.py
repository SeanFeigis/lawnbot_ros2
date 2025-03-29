import rclpy
from ultralytics import YOLO
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_message.msg import MotorData, BoundingBox

# def setup_model():
#     self.get_logger().info('Object Detector Node Started')
#     model = 
#     self.get_logger().info('Object Detector Node Started')
#     return model

MODEL_LOCATION="/home/lawnbot/ROS2/src/lawnbot_ros2/model/best_full_integer_quant_edgetpu.tflite"

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            'image',  # Topic name must match the publisher's topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_LOCATION, task='detect')
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Object Detector Node Started')
        
        self.publisher = self.create_publisher(
            BoundingBox,
            'bound_box',
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info('Received Image from PiCamera')
        try:
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            results = self.model.predict(cv_image, imgsz=640)
            for result in results:
                #self.get_logger().info(str(result.boxes))
                result.save(filename=f"result.jpg")
                boxes = result.boxes.xywh  # Get the bounding boxes
                for box in boxes:
                    bbox = BoundingBox(center_x = box[0].item(), 
                                       center_y = box[1].item(),
                                       width    = box[2].item(),
                                       height   = box[3].item())
                    
                    self.get_logger().info(f'Publishing: {str(bbox)}')
                    self.publisher.publish(bbox)
                    
                    
        except Exception as e:
            self.get_logger().error(f'Failed to predict model: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
