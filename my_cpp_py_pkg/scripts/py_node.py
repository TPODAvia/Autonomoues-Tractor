#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

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
        self.pwm = GPIO.PWM(18, 100)
        self.pwm.start(0)

    def listener_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        # Use the velocities to control the PWM signal
        # This is just an example, adjust as necessary for your specific robot
        duty_cycle = (linear_vel + angular_vel) / 2 * 100
        self.pwm.ChangeDutyCycle(duty_cycle)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()

    rclpy.spin(cmd_vel_subscriber)

    # Destroy the node explicitly
    cmd_vel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
