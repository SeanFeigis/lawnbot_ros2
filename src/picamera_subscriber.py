# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('image_subscriber')
#         self.subscription = self.create_subscription(
#             Image,
#             'image_topic',
#             self.listener_callback,
#             10)
#         self.subscription  # Prevent unused variable warning
#         self.bridge = CvBridge()

#     def listener_callback(self, msg):
#         try:
#             # Convert ROS Image message to OpenCV image
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
#             cv2.imshow('PiCamera Image', cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
#             cv2.waitKey(1)  # Required to keep the window responsive
#         except Exception as e:
#             self.get_logger().error(f"Error converting image: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ImageSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_topic',  # Topic name must match the publisher's topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.subscription  # Prevent unused variable warning
        self.get_logger().info('Image Subscriber Node Started')

    def listener_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Generate a unique filename based on current time
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            filename = f'captured_image_{timestamp}.jpg'

            # Save the image to file
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image as {filename}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to save image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
