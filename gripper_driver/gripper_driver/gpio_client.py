import rclpy
import sys
from std_srvs.srv import SetBool
from rclpy.node import Node

GRIPPER_L = 'gripper_left'
GRIPPER_R = 'gripper_right'

class GPIOControlClient(Node):
    def __init__(self):
        super().__init__('gpio_control_client')
        self.client_left = self.create_client(SetBool, GRIPPER_L)
        self.client_right = self.create_client(SetBool, GRIPER_R)

    def write_gpio_pin_left(self, pin_state):
        request = SetBool.Request()
        request.data = pin_state

        while not self.client_left.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        future = self.client_left.call_async(request)
        rclpy.spin_until_future_complete(self, future)
 
        if future.result() is not None:
            self.get_logger().info('GPIO pin toggled successfully')
        else:
            self.get_logger().error('Failed to toggle GPIO pin')
                                         
    def write_gpio_pin_right(self, pin_state):   
        request = SetBool.Request()
        request.data = pin_state
 
        while not self.client_right.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
 
        future = self.client_right.call_async(request)
        rclpy.spin_until_future_complete(self, future)
 
        if future.result() is not None:
            self.get_logger().info('GPIO pin toggled successfully')
        else:
            self.get_logger().error('Failed to toggle GPIO pin')
 
    def write_gpio_pins(self, pin_state):        
        request = SetBool.Request()
        request.data = pin_state     
                                                                         
        while not self.client_left.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
                    
        future_left = self.client_left.call_async(request)
        future_right = self.client_right.call_async(request)
        rclpy.spin_until_future_complete(self, future_right)
          
        if future.result() is not None:
            self.get_logger().info('GPIO pin toggled successfully')
        else:                                                                                                                                                                                                                                 
            self.get_logger().error('Failed to toggle GPIO pin') 

    def main():
        rclpy.init()
        gpio_client = GPIOControlClient()
    
        grpper = input("Enter gripper you want to control (left, right, both): ")
        assert gripper in ['left', 'right', 'both'], f"{Fore.RED}Invalid gripper name: {gripper}, should be one of: [left, right, both]{Fore.RESET}"
    
        state = int(input("Enter desired state [0 (closed), 1(opened)]: "))
        assert state in [0, 1], f"{Fore.RED}Invalid state: {state}, should be one of: [0, 1]{Fore.RESET}"
    
        if gripper == "left":
            gpio_client.write_gpio_pin_left(state)
        elif gripper == "right":
            gpio_client.write_gpio_pin_right(state)
        else:
            gpio_client.write_gpio_pins(state)
        gpio_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
