import rclpy
import lgpio
from rclpy.node import Node

from std_srvs.srv import Trigger

LIGHT_GPIO = 20
GPIO_NUMBER = 4
ON = 1
OFF = 0
TIMER_PERIOD = 0.5

def open_gpio():
    """Set up the GPIO Connection."""
    h = lgpio.gpiochip_open(GPIO_NUMBER)
    return h

class LightController(Node):

    def __init__(self):
        super().__init__('light_controller')
        self.srv = self.create_service(Trigger, 'light_trigger_service', self.trigger_callback)
        self.get_logger().info('Trigger service server is ready')
        self.h = open_gpio()
        self.light_state = False

    def trigger_callback(self, request, response):
        """Service callback to trigger the light."""
        # This callback is triggered when the client calls the service
        self.get_logger().info('Light Trigger service called')
        
        ### Logic for toggling the light
        if self.light_state:
            self.turn_off_light()
        else:
            self.turn_on_light()
            
        self.light_state = not self.light_state
        
        response.success = True
        response.message = "Light Action triggered successfully"
        return response
    
    def turn_on_light(self):
        """Turn on the light by writing to the GPIO pin."""
        lgpio.gpio_write(self.h, LIGHT_GPIO, ON)
        self.get_logger().info('Light turned ON')
        
    def turn_off_light(self):
        """Turn off the light by writing to the GPIO pin."""
        lgpio.gpio_write(self.h, LIGHT_GPIO, OFF)
        self.get_logger().info('Light turned OFF')
        
    def close_gpio(self):
        """Close the GPIO Connection."""
        self.turn_off_light()
        lgpio.gpiochip_close(self.h)


def main(args=None):
    rclpy.init(args=args)

    light_controller = LightController()

    try:
        rclpy.spin(light_controller)
    except KeyboardInterrupt:
        # Handle the Ctrl+C (SIGINT) gracefully
        light_controller.get_logger().info("Caught Ctrl+C, shutting down light controller.")
        light_controller.close_gpio()  # Call cleanup before shutting down
    finally:
        light_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
