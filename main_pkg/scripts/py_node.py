#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import math
import time
import asyncio

TIMEOUT = 0.5
class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT)
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(19, GPIO.OUT)
        self.pwm18 = GPIO.PWM(18, 100)
        self.pwm12 = GPIO.PWM(12, 100)
        self.pwm13 = GPIO.PWM(13, 100)
        self.pwm19 = GPIO.PWM(19, 100)
        self.pwm18.start(0)
        self.pwm12.start(0)
        self.pwm13.start(0)
        self.pwm19.start(0)
        self.linear_vel = 0
        self.angular_vel = 0

        self.last_msg_time = time.time()
        # self.create_task(self.check_timeout())

    def listener_callback(self, msg):

        # We use the linear_x-linear_vel to slowi the changes of the speed
        # The constant 0.1 is represent how fast the changes is
        self.linear_vel  = (msg.linear.x - self.linear_vel)*0.1
        self.angular_vel = -(msg.angular.z - self.angular_vel)*0.1
        l_distance = 1

        wheel_left  = self.linear_vel - self.angular_vel*l_distance/2
        wheel_right = self.linear_vel + self.angular_vel*l_distance/2

        # Use the velocities to control the PWM signal
        # We use the first order transer function to convert from 0->inf to 0->1:
        # y=(1-e^(-|x|/2))*100
        if wheel_left < 0:
            self.pwm18.ChangeDutyCycle((1-math.exp(-abs(wheel_left)/2))*100)
            self.pwm12.ChangeDutyCycle(0)
        else:
            self.pwm18.ChangeDutyCycle(0)
            self.pwm12.ChangeDutyCycle((1-math.exp(-abs(wheel_left)/2))*100)

        if wheel_right < 0:
            self.pwm19.ChangeDutyCycle(0)
            self.pwm13.ChangeDutyCycle((1-math.exp(-abs(wheel_right)/2))*60)
        else:
            self.pwm19.ChangeDutyCycle((1-math.exp(-abs(wheel_right)/2))*60)
            self.pwm13.ChangeDutyCycle(0)

        self.last_msg_time = time.time()

    # async def check_timeout(self):
    #     while rclpy.ok():
    #         if time.time() - self.last_msg_time > TIMEOUT:
    #             self.pwm18.ChangeDutyCycle(0)
    #             self.pwm12.ChangeDutyCycle(0)
    #             self.pwm13.ChangeDutyCycle(0)
    #             self.pwm19.ChangeDutyCycle(0)
    #         await asyncio.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()

    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
