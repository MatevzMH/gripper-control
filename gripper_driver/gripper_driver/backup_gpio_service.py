from std_srvs.srv import SetBool
import rclpy
from rclpy.node import Node
from gpiozero import LED
import time

GPIO_PIN_L_1_a = LED(2)
GPIO_PIN_L_1_b = LED(3)
GPIO_PIN_L_2_a = LED(17)
GPIO_PIN_L_2_b = LED(27)
GPIO_PIN_R_1_a = LED(10)
GPIO_PIN_R_1_b = LED(9)
GPIO_PIN_R_2_a = LED(5)
GPIO_PIN_R_2_b = LED(6)

SERVICE_NAME_LEFT="gripper_left"
SERVICE_NAME_RIGHT="gripper_right"

DELAY = 4

class GPIOControlNode(Node):
    def __init__(self):
        super().__init__('gpio_control_node')
        self.service_left= self.create_service(SetBool, SERVICE_NAME_LEFT, self.gpio_control_callback_L)
        self.service_right = self.create_service(SetBool, SERVICE_NAME_RIGHT, self.gpio_control_callback_R)

    def gpio_control_callback_L(self, request, response):
        if request.data:
            self.open_gripper("left")
            response.success = True
            response.message = f"L gripper OPENED"
        else:
            self.close_gripper("left")
            response.success = True
            response.message = f"L gripper CLOSED"
        return response

    def gpio_control_callback_R(self, request, response):
        if request.data:
            self.open_gripper("right")
            response.success = True
            response.message = f"R gripper OPENED"
        else:
            self.close_gripper("right")
            response.success = True
            response.message = f"R gripper CLOSED"
        return response

    def open_gripper(self, side:str):
        sides = ["left", "right"]
        assert side in sides
        if side == "left":
            GPIO_PIN_L_1_a.off()
            GPIO_PIN_L_1_b.on()
            GPIO_PIN_L_2_a.off()
            GPIO_PIN_L_2_b.on()
            time.sleep(DELAY)
            GPIO_PIN_L_1_a.off()
            GPIO_PIN_L_1_b.off()
            GPIO_PIN_L_2_a.off()
            GPIO_PIN_L_2_b.off()
        else:
            GPIO_PIN_R_1_a.off()
            GPIO_PIN_R_1_b.on()
            GPIO_PIN_R_2_a.off()
            GPIO_PIN_R_2_b.on()
            time.sleep(DELAY)
            GPIO_PIN_R_1_a.off()
            GPIO_PIN_R_1_b.off()
            GPIO_PIN_R_2_a.off()
            GPIO_PIN_R_2_b.off()
    
    def close_gripper(self, side:str):
        sides = ["left", "right"]
        assert side in sides
        if side == "left":
            GPIO_PIN_L_1_a.on()
            GPIO_PIN_L_1_b.off()
            GPIO_PIN_L_2_a.on()
            GPIO_PIN_L_2_b.off()
            time.sleep(DELAY)
            GPIO_PIN_L_1_a.off()
            GPIO_PIN_L_1_b.off()
            GPIO_PIN_L_2_a.off()
            GPIO_PIN_L_2_b.off()
        else:
            GPIO_PIN_R_1_a.on()
            GPIO_PIN_R_1_b.off()
            GPIO_PIN_R_2_a.on()
            GPIO_PIN_R_2_b.off()
            time.sleep(DELAY)
            GPIO_PIN_R_1_a.off()
            GPIO_PIN_R_1_b.off()
            GPIO_PIN_R_2_a.off()
            GPIO_PIN_R_2_b.off()

def main(args=None):
    rclpy.init()
    gpio_control_node = GPIOControlNode()
    rclpy.spin(gpio_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
