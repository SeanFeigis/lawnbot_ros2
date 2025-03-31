import rclpy
from itertools import cycle
from rclpy.node import Node
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Trigger
from custom_message.msg import MotorData

PATH_TIMER_PERIOD = 1.0  # Seconds

motor_data_array = [
    MotorData(op_code='D', position=6000, speed=600),
    MotorData(op_code='T', position=1800, speed=500),
    MotorData(op_code='D', position=2500, speed=600),
    MotorData(op_code='T', position=1800, speed=500)
]

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Initialize state and publisher
        self._state = State.UNKNOWN
        self.publisher_ = self.create_publisher(MotorData, 'path', 10)
        
        # Service to trigger publishing path
        self.srv = self.create_service(Trigger, 'motion_complete_trigger_service', self.publish_path_service)
        self.get_logger().info('Path Trigger service server is ready')

        # Initialize the path iterator
        self.reset_iterator()

    def reset_iterator(self):
        """Reset the iterator to start from the first motor data command."""
        self.iterator = cycle(motor_data_array)

    def configure(self):
        """Configure: Reset the iterator and prepare for publishing path again."""
        self.get_logger().info("Configuring Path Publisher... Resetting iterator.")
        self.reset_iterator()
        return Transition.TRANSITION_CONFIGURE

    def activate(self):
        """Activate: Start publishing paths after configuration."""
        self.get_logger().info("Activating Path Publisher... Ready to publish paths.")
        self._state = State.ACTIVE
        return Transition.TRANSITION_ACTIVATE

    def deactivate(self):
        """Deactivate: Stop publishing paths."""
        self.get_logger().info("Deactivating Path Publisher... Stopping path publishing.")
        self._state = State.INACTIVE
        return Transition.TRANSITION_DEACTIVATE

    def cleanup(self):
        """Cleanup: Clean up if needed."""
        self.get_logger().info("Cleaning up Path Publisher...")
        self._state = State.INACTIVE
        return Transition.TRANSITION_CLEANUP

    def publish_path_service(self, request, response):
        """Publish the next motor data command on receiving a service request."""
        msg = next(self.iterator)  # Get the next motor data command in the cycle
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {str(msg)}')
        
        response.success = True
        response.message = "Path published successfully."
        return response

def main(args=None):
    rclpy.init(args=args)

    path_publisher = PathPublisher()

    rclpy.spin(path_publisher)

    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
