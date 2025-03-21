import rclpy
from itertools import cycle
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from custom_message.msg import MotorData




motor_data_array = [
        MotorData(op_code='D', position=6000, speed=500),
        MotorData(op_code='T', position=1800, speed=500),
        MotorData(op_code='D', position=1500, speed=500),
        MotorData(op_code='T', position=1800, speed=500),
        
    ]

iterator = cycle(motor_data_array)

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(String, 'path', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_completed_subscription = self.create_subscription(
            Bool,
            'motion_completed',  # Topic name must match the publisher's topic
            self.set_action_completed,
            10)
        
        self.get_completed_subscription  # Prevent unused variable warning
        
        self.action_completed = False
        
    def timer_callback(self):
        if not self.action_completed:
            return
        
        msg = next(iterator)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg))
        
    def set_action_completed(self, msg):
        self.action_completed = msg.data
        if self.action_completed:
            self.get_logger().info('Action Completed')
        else:
            self.get_logger().info('Action Incomplete')
        
def main(args=None):
    rclpy.init(args=args)

    path_publisher = PathPublisher()

    rclpy.spin(path_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
