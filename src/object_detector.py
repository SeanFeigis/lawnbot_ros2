import rclpy
from ultralytics import YOLO
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_message.msg import BoundingBox
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
import time

MODEL_LOCATION = "/home/lawnbot/ROS2/src/lawnbot_ros2/model/best_480_full_integer_quant_edgetpu.tflite"
MODEL_SIZE = 640

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Initialize lifecycle state
        self._state = State.UNKNOWN

        # Set up the subscription to images
        self.subscription = self.create_subscription(
            Image,
            'image',  # Topic name must match the publisher's topic
            self.listener_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.ready = False
        
        self.publisher = self.create_publisher(
            BoundingBox,
            'bound_box',
            10
        )

        self.get_logger().info('Object Detector Node Started')

    def configure(self):
        """Configure: Load the model but don't start inference yet."""
        self.get_logger().info("Configuring Object Detector...")
        self.model = YOLO(MODEL_LOCATION, task='detect')
        self.test_readiness()  # Load the model and check if it's ready
        self.ready = True
        return Transition.TRANSITION_CONFIGURE

    def activate(self):
        """Activate: Start processing images."""
        self.get_logger().info("Activating Object Detector... Inference starts.")
        self._state = State.ACTIVE
        return Transition.TRANSITION_ACTIVATE

    def deactivate(self):
        """Deactivate: Stop processing images."""
        self.get_logger().info("Deactivating Object Detector... Inference stops.")
        self._state = State.INACTIVE
        return Transition.TRANSITION_DEACTIVATE

    def cleanup(self):
        """Cleanup: Cleanup resources if needed."""
        self.get_logger().info("Cleaning up Object Detector...")
        self._state = State.INACTIVE
        self.ready = False
        return Transition.TRANSITION_CLEANUP

    def test_readiness(self):
        """Simple readiness test to check if the model loaded correctly."""
        try:
            dummy_image = np.zeros((MODEL_SIZE, MODEL_SIZE, 3))
            self.model.predict(dummy_image, imgsz=480)
            self.get_logger().info("Model is ready for detection.")
        except Exception as e:
            self.get_logger().error(f"Model failed to load: {e}")
            self.ready = False
    
    def listener_callback(self, msg):
        """Callback for image subscription."""
        if self._state == State.ACTIVE:
            self.get_logger().debug('Received Image from PiCamera')
            try:
                # Convert ROS2 Image message to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                
                results = self.model.predict(cv_image, imgsz=480)

                for result in results:
                    boxes = result.boxes.xywh  # Get the bounding boxes
                    for box in boxes:
                        bbox = BoundingBox(center_x=box[0].item(), 
                                           center_y=box[1].item(),
                                           width=box[2].item(),
                                           height=box[3].item())
                        self.publisher.publish(bbox)
                        self.vid_publisher = self.create_publisher(Image, 'image_annotated', 2)
            except Exception as e:
                self.get_logger().error(f'Failed to predict model: {e}')
    
    def get_state(self):
        return self._state

def main(args=None):
    rclpy.init(args=args)
    object_detector = ObjectDetector()

    # Add lifecycle manager service if needed (not necessary for now, only lifecycle)
    rclpy.spin(object_detector)
    
    object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
