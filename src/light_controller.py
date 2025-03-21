import rclpy
import serial
import time
from rclpy.node import Node

from std_srvs.srv import Trigger

def send_command(ser, command):
    """Send a command to the Kangaroo motor controller."""
    ser.write(command.encode('utf-8') + b'\r')  # Kangaroo commands end with a carriage return
    time.sleep(0.1)
    response = ser.read(ser.in_waiting).decode('utf-8')
    return response

class LightController(Node):

    def __init__(self):
        super().__init__('light_controller')
        self.srv = self.create_service(Trigger, 'nozzle_trigger_service', self.trigger_callback)
        self.get_logger().info('Trigger service server is ready')

    def trigger_callback(self, request, response):
        # This callback is triggered when the client calls the service
        self.get_logger().info('Trigger service called')
        
        ### Logic for triggering GPIO of light
        
        response.success = True
        response.message = "Action triggered successfully"
        return response


def main(args=None):
    rclpy.init(args=args)

    light_controller = LightController()

    rclpy.spin(light_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    light_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
