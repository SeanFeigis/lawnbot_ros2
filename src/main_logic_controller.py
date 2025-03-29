import rclpy
import serial
import time
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger
from custom_message.msg import MotorData, BoundingBox

Y_MIDPOINT =  1296
Y_RANGE = 200

class MainLogicController(Node):

    def __init__(self):
        super().__init__('main_logic_controller')
        ### Path Subscriber
        self.path_subscription = self.create_subscription(
            MotorData,
            'path',
            self.path_callback,
            10)
        
        ### bounding box Subscriber
        self.bound_box_subscription = self.create_subscription(
            BoundingBox,
            'bound_box',
            self.dandelion_detected_callback,
            10)
        
        self.current_path = None
        
        ### Pump Client
        self.pump_client = self.create_client(Trigger, 'pump_trigger_service')
        while not self.pump_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pump service not available, waiting again...')
            
        ### Light Client
        self.light_client = self.create_client(Trigger, 'light_trigger_service')
        while not self.light_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('light service not available, waiting again...')
            
            
        ### Motor Output Publisher 
        self.motor_output_publisher_ = self.create_publisher(MotorData, 'motor_output', 10)
        timer_period = 1.0  # seconds
        #self.motor_controller_timer = self.create_timer(timer_period, self.motor_controller_callback)
        
        self.path_subscription  # prevent unused variable warning
        self.bound_box_subscription
        
        self.setup()
        
    def setup(self):
        self.send_light_request()
        
    def send_light_request(self):
        request = Trigger.Request()
        future = self.light_client.call_async(request)
        future.add_done_callback(self.light_callback)
    ### Light Service Send Trigger Request
    def light_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Service response: {response.message}')
        else:
            self.get_logger().info('Service call failed')    
        
        
    ### Pump Service Send Trigger Request    
    def send_pump_request(self):
        request = Trigger.Request()
        future = self.pump_client.call_async(request)
        future.add_done_callback(self.pump_callback)
        
    ### Pump Service Callback
    def pump_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Service response: {response.message}')
        else:
            self.get_logger().info('Service call failed')

    ### Path Subscriber Handler
    def path_callback(self, msg):
        self.motor_controller_callback(msg)

    ### Motor Controller Publisher Handler
    def motor_controller_callback(self, msg):
        self.current_path = msg
        self.motor_output_publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg))
        
        
    def dandelion_detected_callback(self, msg):
        self.get_logger().info('Dandelion detected: "%s"' % msg.data)
        y_midpoint = msg.center_y
        
        if (Y_MIDPOINT - Y_RANGE) <= y_midpoint <= (Y_MIDPOINT + Y_RANGE):
            self.get_logger().info('Dandelion is within range')
            # Trigger the pump
            self.send_pump_request()
        

def main(args=None):
    rclpy.init(args=args)

    main_logic_controller = MainLogicController()

    rclpy.spin(main_logic_controller)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    main_logic_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
