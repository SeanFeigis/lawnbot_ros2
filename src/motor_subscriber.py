import rclpy
import serial
import time
from rclpy.node import Node

from std_msgs.msg import String

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
    
    # Start command must be sent first
    start_command = "1,start"
    response = send_command(ser, start_command)
    print(f"Response: {response}")
    start_command = "2,start"
    response = send_command(ser, start_command)
    print(f"Response: {response}")
    
    return ser

class PositionSubscriber(Node):

    def __init__(self):
        super().__init__('position_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.ser = setup_serial()
        
    def listener_callback(self, msg):
        position = msg.data
        command = f'1,P{position}'
        response = send_command(self.ser, command)
        print(f"Response: {response}")
        command = f'2,P{position}'
        response = send_command(self.ser, command)
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    position_subscriber = PositionSubscriber()

    rclpy.spin(position_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    position_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
