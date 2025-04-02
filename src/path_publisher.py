import rclpy
from itertools import cycle
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, Empty
from custom_message.msg import MotorData


PATH_TIMER_PERIOD = 1.0 # Seconds

motor_data_array = [
        MotorData(op_code='D', position=6000, speed=600),
        MotorData(op_code='T', position=1800, speed=500),
        MotorData(op_code='D', position=2500, speed=600),
        MotorData(op_code='T', position=1800, speed=500)
    ]

class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.publisher_ = self.create_publisher(MotorData, 'path', 10)
        
        self.get_completed_subscription = self.create_subscription(
            Bool,
            'motion_completed',  # Topic name must match the publisher's topic
            self.set_action_completed,
            10)
        
        self.iterator = cycle(motor_data_array)
        
        self.get_completed_subscription  # Prevent unused variable warning
        self.action_completed = False #Does not give first instruction until response from motor controller
        
        self.cli = self.create_client(Trigger, 'model_ready')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Model service not available, waiting...")
            
        self.check_model_status()
        
        self.start_service = self.create_service(Empty, 'path_start', self.start_path_callback)
        self.start_service = self.create_service(Empty, 'path_stop', self.stop_path_callback)
        
        self.timer = self.create_timer(PATH_TIMER_PERIOD, self.timer_callback, autostart=False)
        
        
    def check_model_status(self):
        req = Trigger.Request()
        while rclpy.ok():
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info("Model is ready. Proceeding with path node execution.")
                break
            self.get_logger().info("Model not ready, retrying...")
            
    def timer_callback(self):
        if not self.action_completed:
            return
        
        msg = next(self.iterator)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg))
        
    def set_action_completed(self, msg):
        self.action_completed = msg.data
        if self.action_completed:
            self.get_logger().debug('Action Completed')
        else:
            self.get_logger().debug('Action Incomplete')
            
            
    def stop_path_callback(self, request, response):
        self.timer.cancel()
        response = Empty.Response()
        return response
        
    def start_path_callback(self,request, response):
        self.iterator = cycle(motor_data_array)
        self.timer.reset()
        response = Empty.Response()
        return response
        
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
