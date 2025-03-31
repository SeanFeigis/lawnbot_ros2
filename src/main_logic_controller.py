import rclpy
from rclpy.lifecycle import LifecycleNode, State, Transition
from rclpy.node import Node
from std_srvs.srv import Trigger
from custom_message.msg import MotorData, BoundingBox

Y_MIDPOINT = 500
Y_RANGE = 50

class MainLogicController(LifecycleNode):

    def __init__(self):
        super().__init__('main_logic_controller')
        self.current_path = None
        self.pump_client = None
        self.light_client = None
        self.motor_output_publisher_ = None
        self.path_subscription = None
        self.bound_box_subscription = None

    def configure(self):
        """Configure: Initialize all subscribers, publishers, and clients."""
        self.get_logger().info("Configuring MainLogicController...")

        # Create the pump client
        self.pump_client = self.create_client(Trigger, 'pump_trigger_service')

        # Create the light client
        self.light_client = self.create_client(Trigger, 'light_trigger_service')

        # Create the motor output publisher
        self.motor_output_publisher_ = self.create_publisher(MotorData, 'motor_output', 10)

        return Transition.TRANSITION_CONFIGURE

    def activate(self):
        """Activate: Set up the logic for the node when activated."""
        self.get_logger().info("Activating MainLogicController...")

        # Re-create the subscribers when activating
        self.path_subscription = self.create_subscription(
            MotorData,
            'path',
            self.path_callback,
            10)

        self.bound_box_subscription = self.create_subscription(
            BoundingBox,
            'bound_box',
            self.dandelion_detected_callback,
            10)

        # Perform setup logic directly during activation since we know the node is active
        self.send_light_request()

        return Transition.TRANSITION_ACTIVATE

    def send_light_request(self):
        """ Send a request to the light service to turn on the light. """
        request = Trigger.Request()
        future = self.light_client.call_async(request)
        future.add_done_callback(self.light_callback)

    def light_callback(self, future):
        """ Callback for the light service request."""
        response = future.result()
        if response.success:
            self.get_logger().debug(f'Light Service response: {response.message}')
        else:
            self.get_logger().debug('Light Service call failed')

    def send_pump_request(self):
        """ Send a request to the pump service to turn on the pump. """
        request = Trigger.Request()
        future = self.pump_client.call_async(request)
        future.add_done_callback(self.pump_callback)

    def pump_callback(self, future):
        """ Callback for the pump service request."""
        response = future.result()
        if response.success:
            self.get_logger().info(f'Pump Service response: {response.message}')
        else:
            self.get_logger().info('Pump Service call failed')

    def path_callback(self, msg):
        """Forward the path subscriber data to the motor controller."""
        self.motor_controller_callback(msg)

    def motor_controller_callback(self, msg):
        """ Publish the motor data to the motor_output topic"""
        self.current_path = msg
        self.motor_output_publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{str(msg)}"')

    def dandelion_detected_callback(self, msg):
        """Callback function for the bounding box subscriber."""
        self.get_logger().debug('Dandelion detected')
        y_midpoint = msg.center_y

        if (Y_MIDPOINT - Y_RANGE) <= y_midpoint <= (Y_MIDPOINT + Y_RANGE):
            self.get_logger().info(f'Dandelion is within range at: {y_midpoint}')
            # Trigger the pump
            self.send_pump_request()

    def deactivate(self):
        """Deactivate: Cancel timers and stop functionality."""
        self.get_logger().info("Deactivating MainLogicController...")


        self.send_light_request()
        # Optional: You can also cancel timers here if you have any active timers
        
        return Transition.TRANSITION_DEACTIVATE

    def cleanup(self):
        """Cleanup: Shut down the motor controller and destroy clients and subscribers."""
        self.get_logger().info("Cleaning up MainLogicController...")

        # Destroy the service clients
        if self.pump_client:
            self.pump_client.destroy()
        if self.light_client:
            self.light_client.destroy()

        # Destroy the publisher
        if self.motor_output_publisher_:
            self.motor_output_publisher_.destroy()

        # Destroy the subscribers
        if self.path_subscription:
            self.path_subscription.destroy()
        if self.bound_box_subscription:
            self.bound_box_subscription.destroy()

        return Transition.TRANSITION_CLEANUP


def main(args=None):
    rclpy.init(args=args)

    main_logic_controller = MainLogicController()

    rclpy.spin(main_logic_controller)

    # Destroy the node explicitly
    main_logic_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
