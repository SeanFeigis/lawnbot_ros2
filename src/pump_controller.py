import rclpy
import serial
import time
import RPi.GPIO as GPIO
from rclpy.node import Node

from std_msgs.msg import String
from lawnbot_ros2.msg import MotorData
from std_srvs.srv import Trigger

def setup_gpio():
    """Set up the GPIO Connection."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    

class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        
        self.srv = self.create_service(Trigger, 'pump_trigger_service', self.pump_trigger_callback)
        self.get_logger().info('Pump Trigger service server is ready')
        self.pump_state = False
        setup_gpio()

    def pump_trigger_callback(self, request, response):
        # This callback is triggered when the client calls the service
        self.get_logger().info('Trigger service called')
        
        self.pump_state = not self.pump_state
        
        ### Logic for triggering GPIO of pump
        GPIO.output(23, GPIO.HIGH if self.pump_state else GPIO.LOW)
        
        state_str = "HIGH" if self.pump_state else "LOW"
        self.get_logger().info(f'Trigger service called - Pin 23 set to {state_str}')
        
        response.success = True
        response.message = f"Action triggered successfully: Pin 23 set to {state_str}"
        return response

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
