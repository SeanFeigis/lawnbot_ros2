import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Empty  # SetBool for wrapper, Empty for service calls

class WrapperNode(Node):
    def __init__(self):
        super().__init__('start_stop_wrapper')

        # Create the wrapper service that accepts start/stop requests
        self.srv = self.create_service(SetBool, 'wrapper_service', self.wrapper_callback)

        # Create Empty service clients for service_a and service_b
        self.motor_start = self.create_client(Empty, 'motor_start')
        self.motor_stop = self.create_client(Empty, 'motor_stop')
        self.path_start = self.create_client(Empty, 'path_start')
        self.path_stop = self.create_client(Empty, 'path_stop')

    def wrapper_callback(self, request, response):
        """Handles start/stop requests and calls the respective services."""
        command = request.data  # True = start, False = stop
        action = "starting" if command else "stopping"
        self.get_logger().info(f"Received request: {action} services.")

        if command:
            self.trigger_service(self.motor_start, self.path_start)
        else:
            self.trigger_service(self.motor_stop, self.path_stop)

        response.success = True
        response.message = f"Sent {action} command."
        return response

    def trigger_service(self, client_1, client_2):
        """Helper function to call an Empty service."""
        if not client_1.wait_for_service(timeout_sec=1.0):
            pass
        else:
            client_1.call_async(Empty.Request())
        if not client_2.wait_for_service(timeout_sec=1.0):
            pass
        else:
            client_2.call_async(Empty.Request())
            
def main(args=None):
    rclpy.init(args=args)
    node = WrapperNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
