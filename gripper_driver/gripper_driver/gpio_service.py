#!/usr/bin/env python3
from std_srvs.srv import SetBool
from sitestruct_ros2_interfaces.srv import Gripper
import rclpy

# from colorama import Fore
from rclpy.node import Node
import RPi.GPIO as GPIO
import time

# GPIO_PIN_L_a = 17
# GPIO_PIN_L_b = 27
# GPIO_PIN_R_a = 2
# GPIO_PIN_R_b = 3

GPIO_PIN_L_1_a = 2
GPIO_PIN_L_1_b = 3
GPIO_PIN_L_2_a = 17
GPIO_PIN_L_2_b = 27
GPIO_PIN_R_1_a = 10
GPIO_PIN_R_1_b = 9
GPIO_PIN_R_2_a = 5
GPIO_PIN_R_2_b = 6

SERVICE_NAME_LEFT = "gripper_left_open"
SERVICE_NAME_RIGHT = "gripper_right_open"
PARAMETER_NAME = "gripper_delay"


class GPIOControlNode(Node):
    def __init__(self):
        super().__init__("gpio_control_node")  # type: ignore
        DEFAULT_DELAY = 4.0
        self.delay = DEFAULT_DELAY
        self.service_left = self.create_service(
            Gripper, SERVICE_NAME_LEFT, self.gpio_control_callback_L
        )
        self.service_right = self.create_service(
            Gripper, SERVICE_NAME_RIGHT, self.gpio_control_callback_R
        )
        # self.declare_parameter(PARAMETER_NAME, DEFAULT_DELAY)
        GPIO.setmode(GPIO.BCM)

        GPIO.setup(GPIO_PIN_L_1_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_L_1_b, GPIO.OUT)
        GPIO.setup(GPIO_PIN_L_2_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_L_2_b, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_1_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_1_b, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_2_a, GPIO.OUT)
        GPIO.setup(GPIO_PIN_R_2_b, GPIO.OUT)

    def gpio_control_callback_L(self, request, response):
        self.delay = request.delay
        if 0.1 <= request.delay <= 5:
            self.delay = request.delay
            self.get_logger().info(f"Delay param: {self.delay}")
        else:
            # Handle invalid delay value by raising an error or setting a default
            response.success = False
            response.message = "Delay must be between 0.1 and 5."
            print(f"Invalid delay parameter: {request.delay}")
            return response

        if request.open:
            self.open_gripper("left")
            response.success = True
            response.message = "L gripper OPENED"
        else:
            self.close_gripper("left")
            response.success = True
            response.message = "L gripper CLOSED"
        return response

    def gpio_control_callback_R(self, request, response):
        if 0.1 <= request.delay <= 5:
            self.delay = request.delay
            self.get_logger().info(f"Delay param: {self.delay}")
        else:
            # Handle invalid delay value by raising an error or setting a default
            response.success = False
            response.message = "Delay must be between 0.1 and 5."
            print(f"Invalid delay parameter: {request.delay}")
            return response
        print(f"Delay param: {self.delay}")
        if request.open:
            self.open_gripper("right")
            response.success = True
            response.message = "R gripper OPENED"
        else:
            self.close_gripper("right")
            response.success = True
            response.message = "R gripper CLOSED"
        return response


    def open_gripper(self, side: str):
        sides = ["left", "right"]
        assert side in sides
        if side == "left":
            GPIO.output(GPIO_PIN_L_1_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_L_1_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_2_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_2_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_1_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_1_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_2_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_2_b, GPIO.LOW)
        elif side == "right":
            GPIO.output(GPIO_PIN_R_1_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_1_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_2_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_2_b, GPIO.HIGH)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_R_1_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_1_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_2_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_2_b, GPIO.LOW)
        else:
            raise Warning("Must be right of left")

    def close_gripper(self, side: str):
        sides = ["left", "right", "both"]
        assert side in sides
        if side == "left":
            GPIO.output(GPIO_PIN_L_1_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_1_b, GPIO.HIGH)
            GPIO.output(GPIO_PIN_L_2_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_L_2_b, GPIO.LOW)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_L_1_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_1_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_2_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_L_2_b, GPIO.LOW)
        elif side == "right":
            GPIO.output(GPIO_PIN_R_1_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_1_b, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_2_a, GPIO.HIGH)
            GPIO.output(GPIO_PIN_R_2_b, GPIO.LOW)
            time.sleep(self.delay)
            GPIO.output(GPIO_PIN_R_1_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_1_b, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_2_a, GPIO.LOW)
            GPIO.output(GPIO_PIN_R_2_b, GPIO.LOW)
        else:
            raise Warning("Must be right of left")


def main(args=None):
    rclpy.init()
    gpio_control_node = GPIOControlNode()
    rclpy.spin(gpio_control_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
