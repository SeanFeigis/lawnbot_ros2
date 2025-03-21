import rclpy
import serial
import time
from rclpy.node import Node

from std_msgs.msg import String
from custom_message.msg import MotorData
from std_msgs.msg import Bool


GETPOS_COMMAND = "D,getp"

def send_command(ser, command):
    """Send a command to the Kangaroo motor controller."""
    ser.write(command.encode('utf-8') + b'\r')  # Kangaroo commands end with a carriage return
    time.sleep(0.1)
    response = ser.read(ser.in_waiting).decode('utf-8')
    return response

def setup_serial():
    """Set up the serial connection and send the start command."""
    ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=1)
    time.sleep(2)  # Wait for the serial connection to establish
    print("Connected to Kangaroo on /dev/ttyUSB0")
    
    # Start command must be sent first
    start_command = "1,start"
    response = send_command(ser, start_command)
    print(f"Response: {response}")
    start_command = "2,start"
    response = send_command(ser, start_command)
    print(f"Response: {response}")
    
    return ser

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = setup_serial()
        
        self.motion_complete_publisher_ = self.create_publisher(Bool, 'motion_completed', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_completion_status_timer_callback)
            
    def get_completion_status(self):
        command = GETPOS_COMMAND
        response = send_command(self.ser, command)
        return 'P' in response  
            
    def publish_completion_status_timer_callback(self):
        msg = Bool()
        
        msg.data = self.get_completion_status()

        self.motion_completed_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%b"' % msg.data)
            
    def listener_callback(self, msg):
        MotorData = msg.data
        command = f'{MotorData.op_code},P{MotorData.position},S{MotorData.speed}'
        
        response = send_command(self.ser, command)
        print(f"Response: {response}")
        
        self.get_logger().info('I heard: "%s"' % String(msg))


def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    rclpy.spin(motor_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
