import rclpy
from rclpy.node import Node
from rclpy.lifecycle import ChangeState
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')

        # Node dependencies (activation order)
        self.object_detector = "/object_detector"
        self.picamera_controller = "/picamera_controller"
        self.main_logic_controller = "/main_logic_controller"
        self.motor_controller = "/motor_controller"
        self.path_publisher = "/path_publisher"  # Activates last!

        self.get_logger().info("Lifecycle Manager Node started.")
        
        # Call configure_nodes() when the node starts
        self.configure_nodes()

        # Service to change the lifecycle state of this manager
        self.lifecycle_service = self.create_service(
            ChangeState,
            '/lifecycle_manager/change_state',
            self.handle_change_state_request
        )
        
    def change_state(self, node_name, transition_id):
        """Send a lifecycle transition request to a node."""
        self.get_logger().info(f"Transitioning {node_name} to {transition_id}...")
        client = self.create_client(ChangeState, f'{node_name}/change_state')

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service not available for {node_name}")
            return False

        request = ChangeState.Request()
        request.transition.id = transition_id

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result() and future.result().success

    def change_nodes_state(self, transition_id):
        """Change state of all nodes according to the provided transition."""
        nodes = [
            self.object_detector,
            self.picamera_controller,
            self.main_logic_controller,
            self.motor_controller,
            self.path_publisher
        ]
        
        for node in nodes:
            if not self.change_state(node, transition_id):
                self.get_logger().error(f"{node} failed to transition.")
                return

    def handle_change_state_request(self, request, response):
        """Handle lifecycle state change requests."""
        transition_id = request.transition.id

        if transition_id in [Transition.TRANSITION_ACTIVATE, 
                             Transition.TRANSITION_CONFIGURE, 
                             Transition.TRANSITION_DEACTIVATE, 
                             Transition.TRANSITION_SHUTDOWN]:
            self.get_logger().info(f"Received request to change state to {transition_id}.")
            self.change_nodes_state(transition_id)
        else:
            self.get_logger().error(f"Unsupported transition: {transition_id}")
            response.success = False
            response.message = f"Unsupported transition {transition_id}"
            return response

        response.success = True
        response.message = f"Transition to {transition_id} completed."
        return response

def main(args=None):
    rclpy.init(args=args)
    manager = LifecycleManager()
    rclpy.spin(manager)  # Keeps the node alive to handle lifecycle service requests
    rclpy.shutdown()

if __name__ == '__main__':
    main()
