import rclpy
import serial
import time
from rclpy.node import Node

from std_msgs.msg import String
from custom_message.msg import MotorData
from std_msgs.msg import Bool


GETPOS_COMMAND_DRIVE = "D,getp"
GETPOS_COMMAND_TURN = "T,getp"

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

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            MotorData,
            'motor_output',
            self.motor_output_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = setup_serial()
        
        self.motion_complete_publisher_ = self.create_publisher(Bool, 'motion_completed', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_completion_status_timer_callback)
            
    def get_completion_status(self):
        """Get the completion status of the motor."""
        response_drive = send_command(self.ser, GETPOS_COMMAND_DRIVE)
        
        response_turn = send_command(self.ser, GETPOS_COMMAND_TURN)
        return 'P' in response_drive and 'P' in response_turn
            
    def publish_completion_status_timer_callback(self):
        msg = Bool()
        
        msg.data = self.get_completion_status()

        self.motion_completed_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%b"' % msg.data)
            
    def motor_output_callback(self, msg):
        MotorData = msg
        command = f'{MotorData.op_code},PI{MotorData.position}S{MotorData.speed}'
         
        self.get_logger().info(f'Sending to Serial: {command}')
        
        response = send_command(self.ser, command)
        print(f"Response: {response}")


    def stop_and_shutdown(self):
        send_command(self.ser, "D,S0")
        send_command(self.ser, "T,S0")
        self.ser.close()
        self.get_logger().info('Stopping and closing serial connection')

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.stop_and_shutdown()
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
