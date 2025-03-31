import rclpy
import serial
import time
from rclpy.node import Node
from lifecycle_msgs.msg import State
from lifecycle_msgs.msg import Transition
from std_srvs.srv import Trigger
from custom_message.msg import MotorData

GETPOS_COMMAND_DRIVE = "D,getp"
GETPOS_COMMAND_TURN = "T,getp"
GET_COMPLETED_ACTION_TIMER_PERIOD = 0.1  # seconds

def send_command(ser, command):
    """Send a command to the Kangaroo motor controller."""
    ser.write(command.encode('utf-8') + b'\r')  # Kangaroo commands end with a carriage return
    time.sleep(0.1)
    response = ser.read(ser.in_waiting).decode('utf-8')
    return response

def setup_serial():
    """Set up the serial connection and send the start command."""
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    time.sleep(2)  # Wait for the serial connection to establish
    print("Connected to Kangaroo on /dev/ttyUSB0")
    
    # Send start commands to initialize the motor controller
    start_commands = ["D,start", "T,start", "D,p0", "T,p0"]
    for command in start_commands:
        response = send_command(ser, command)
        print(f"Response for '{command}': {response}")
    
    return ser

from rclpy.lifecycle import LifecycleNode, State
from rclpy.node import Node

class MotorController(LifecycleNode):

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = None
        self.ser = None
        self.motion_complete_client = None
        self.timer = None

    def configure(self):
        """Configure: Initialize the serial connection and create subscriber and timer."""
        self.get_logger().info("Configuring MotorController...")

        # Set up the serial connection
        self.ser = setup_serial()

        # Set up the motion complete service client
        self.motion_complete_client = self.create_client(Trigger, 'motion_complete_trigger_service')
        
        # Create the subscriber in the configure phase
        self.subscription = self.create_subscription(
            MotorData,
            'motor_output',
            self.motor_output_callback,
            10)
        
        # Create the timer in the configure phase, but don't start it yet
        self.timer = self.create_timer(GET_COMPLETED_ACTION_TIMER_PERIOD, self.publish_completion_status_timer_callback, autostart = False)

        return Transition.TRANSITION_CONFIGURE

    def activate(self):
        """Activate: Start the full functionality of the motor controller and reset the timer."""
        self.get_logger().info("Activating MotorController...")

        # Reset and start the timer
        if self.timer:
            self.timer.reset()

        return Transition.TRANSITION_ACTIVATE

    def deactivate(self):
        """Deactivate: Stop sending commands and disable the timer."""
        self.get_logger().info("Deactivating MotorController...")

        # Cancel the completion status timer when deactivating
        if self.timer:
            self.timer.cancel()

        # Tell the motor to stop
        self.stop_motors()
        
        return Transition.TRANSITION_DEACTIVATE

    def cleanup(self):
        """Cleanup: Shut down the motor controller and close the serial connection."""
        self.get_logger().info("Cleaning up MotorController...")

        if self.ser:
            self.stop_and_shutdown()

        # Clean up the service client
        if self.motion_complete_client:
            self.motion_complete_client.destroy()
        
        return Transition.TRANSITION_CLEANUP
    
    def stop_motors(self):
        """Stop the motors by sending the stop command."""
        send_command(self.ser, "D,S0")
        send_command(self.ser, "T,S0")
        self.get_logger().info("Motors stopped.")

    def stop_and_shutdown(self):
        """Stop the motor and shutdown the serial connection."""
        self.stop_motors()
        self.ser.close()
        self.get_logger().info('Motor stopped and serial connection closed.')

    def get_completion_status(self):
        """Get the completion status of the motor."""
        response_drive = send_command(self.ser, GETPOS_COMMAND_DRIVE)
        response_turn = send_command(self.ser, GETPOS_COMMAND_TURN)
        return 'P' in response_drive and 'P' in response_turn

    def publish_completion_status_timer_callback(self):
        """Publish the completion status of the motor."""
        # Check if the node is in ACTIVE state before proceeding
        if self.get_state().id == State.ACTIVE:
            if self.get_completion_status():
                request = Trigger.Request()
                future = self.motion_complete_client.call_async(request)
                future.add_done_callback(self.motion_complete_client)
                self.get_logger().debug('Triggering Completed motion service')

    def motor_output_callback(self, msg):
        """Send motor commands based on subscriber message."""
        # Check if the node is in ACTIVE state
        if self.get_state().id == State.ACTIVE:
            command = f'{msg.op_code},PI{msg.position}S{msg.speed}'
            self.get_logger().info(f'Sending to Serial: {command}')
            response = send_command(self.ser, command)
            self.get_logger().info(f"Response: {response}")
        else:
            self.get_logger().info("MotorController is not ACTIVE, ignoring command.")


def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()
    
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        # Handle the Ctrl+C (SIGINT) gracefully
        motor_controller.get_logger().info("Caught Ctrl+C, shutting down motor controller.")
        motor_controller.stop_and_shutdown()
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
