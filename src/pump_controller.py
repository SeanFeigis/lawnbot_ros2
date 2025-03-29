import rclpy
import lgpio
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

PUMP_GPIO = 16
GPIO_NUMBER = 4
ON = 1
OFF = 0
TIMER_PERIOD = 0.5

def open_gpio():
    """Set up the GPIO Connection."""
    h = lgpio.gpiochip_open(GPIO_NUMBER)
    return h

class PumpController(Node):

    def __init__(self):
        super().__init__('pump_controller')
        self.srv = self.create_service(Trigger, 'pump_trigger_service', self.pump_trigger_callback)
        self.get_logger().info('Pump Trigger service server is ready')
        self.pump_shutoff_timer = self.create_timer(TIMER_PERIOD, self.timer_callback, autostart = False)
        #self.pump_state = False
        self.h = open_gpio()

    def pump_trigger_callback(self, request, response):
        # This callback is triggered when the client calls the service
        self.get_logger().info('Trigger service called')
        
        self.turn_on_pump()
        self.pump_shutoff_timer.reset()
        
        response.success = True
        response.message = f"Action triggered successfully: Pump Triggered"
        return response
    
    def turn_on_pump(self):
        lgpio.gpio_write(self.h, PUMP_GPIO, ON)
        self.get_logger().info('Pump turned ON')
        
    def turn_off_pump(self):
        lgpio.gpio_write(self.h, PUMP_GPIO, OFF)
        self.get_logger().info('Pump turned OFF')
        
    def timer_callback(self):
        """Callback function to turn off the pump after a certain period."""
        self.get_logger().info('Timer triggered - turning off pump')
        self.turn_off_pump()
        
        # Stop the timer
        self.pump_shutoff_timer.stop()
        
        
    def close_gpio(self):
        """Close the GPIO Connection."""
        self.turn_off_pump()
        lgpio.gpiochip_close(self.h)

def main(args=None):
    rclpy.init(args=args)

    pump_controller = PumpController()

    try:
        rclpy.spin(pump_controller)
    except KeyboardInterrupt:
        # Handle the Ctrl+C (SIGINT) gracefully
        pump_controller.get_logger().info("Caught Ctrl+C, shutting down light controller.")
        pump_controller.close_gpio()  # Call cleanup before shutting down
    finally:
        pump_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
