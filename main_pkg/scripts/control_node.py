#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import math
import time

TIMEOUT = 1
class CmdVelSubscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback,10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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
        self.linear_msg = 0

        self.angular_vel = 0
        self.angular_msg = 0

        # low pass parameter
        self.alpha = 0.2
        self.beta = 35
        self.filtered_value = 10
        self.l_distance = 1
        self.last_msg_time = time.time()

    def listener_callback(self, msg):
        self.linear_msg = msg.linear.x
        self.angular_msg = msg.angular.z
        self.last_msg_time = time.time()
    
    def timer_callback(self):

        if time.time() - self.last_msg_time > TIMEOUT:
            self.linear_msg = 0
            self.angular_msg = 0       

        # We use the low pass filter to slow the changes of the speed
        # The constant alpha is represent how fast the changes is
        self.linear_vel = self.alpha*self.linear_msg + (1 - self.alpha)*self.linear_vel
        self.angular_vel = self.alpha*self.angular_msg + (1 - self.alpha)*self.angular_vel

        wheel_left  = self.linear_vel - self.angular_vel*self.l_distance/2
        wheel_right = self.linear_vel + self.angular_vel*self.l_distance/2

        # print("wheel_left: {}".format(wheel_left))
        # print("wheel_right: {}".format(wheel_right))

        # Use the velocities to control the PWM signal
        # We use the first order transer function to convert from 0->inf to 0->1:
        # y=(1-e^(-|x|*35))*100
        if wheel_left < 0:
            self.pwm18.ChangeDutyCycle((1-math.exp(-abs(wheel_left)*self.beta))*100)
            self.pwm12.ChangeDutyCycle(0)
        else:
            self.pwm18.ChangeDutyCycle(0)
            self.pwm12.ChangeDutyCycle((1-math.exp(-abs(wheel_left)*self.beta))*100)

        if wheel_right < 0:
            self.pwm19.ChangeDutyCycle(0)
            self.pwm13.ChangeDutyCycle((1-math.exp(-abs(wheel_right)*self.beta))*100)
        else:
            self.pwm19.ChangeDutyCycle((1-math.exp(-abs(wheel_right)*self.beta))*100)
            self.pwm13.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()
    try:
        rclpy.spin(cmd_vel_subscriber)
    except:
        pass
    finally:
        cmd_vel_subscriber.destroy_node()

if __name__ == '__main__':
    main()