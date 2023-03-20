"""
The output of this driver is set_cmd_vel function
The data will hook to the "Local Planning" module
"""


import time
import RPi.GPIO as GPIO 
# import threading
# from getkey import getkey, keys

class MotorDriver(object):

    def __init__(self, wheel_distance=0.098, wheel_diameter=0.066, i_BASE_PWM=50, i_MULTIPLIER_STANDARD=0.1, i_MULTIPLIER_PIVOT=1.0, simple_mode = True):
        """
        M1 = Right Wheel
        M2 = Left Wheel
        :param wheel_distance: Distance Between wheels in meters
        :param wheel_diameter: Diameter of the wheels in meters
        """

        self.D1 = 12
        self.D2 = 18
        self.D3 = 19
        self.D4 = 13

        self.PWM1 = 0
        self.PWM2 = 0
        self.BASE_PWM = i_BASE_PWM
        self.MAX_PWM = 100

        self.simple_mode = simple_mode

        # Wheel and chasis dimensions
        self._wheel_distance = wheel_distance
        self._wheel_radius = wheel_diameter / 2.0
        self.MULTIPLIER_STANDARD = i_MULTIPLIER_STANDARD
        self.MULTIPLIER_PIVOT = i_MULTIPLIER_PIVOT

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.D1, GPIO.OUT)
        GPIO.setup(self.D2, GPIO.OUT)
        GPIO.setup(self.D3, GPIO.OUT)
        GPIO.setup(self.D4, GPIO.OUT)
        self.left_pwm1 = GPIO.PWM(self.D1, 500)
        self.left_pwm2 = GPIO.PWM(self.D2, 500)
        self.right_pwm1 = GPIO.PWM(self.D3, 500)
        self.right_pwm2 = GPIO.PWM(self.D4, 500)

        self.left_pwm1.start(self.PWM1)
        self.left_pwm2.start(self.PWM1)
        self.right_pwm1.start(self.PWM2)
        self.right_pwm2.start(self.PWM2)

    def __del__(self):
        GPIO.cleanup()

    # def set_motor(self, A1, A2, B1, B2):
    #     GPIO.output(self.PWMA1, A1)
    #     GPIO.output(self.PWMA2, A2)
    #     GPIO.output(self.PWMB1, B1)
    #     GPIO.output(self.PWMB2, B2)

    def stop(self):
        self.left_pwm1.stop(0)
        self.left_pwm2.stop(0)
        self.right_pwm1.stop(0)
        self.right_pwm2.stop(0)
        GPIO.output(self.D1,0)
        GPIO.output(self.D2,0)
        GPIO.output(self.D3,0)
        GPIO.output(self.D4,0)

    def forward(self, left_speed, right_speed):
        self.left_pwm2.stop(0)
        self.right_pwm2.stop(0)
        GPIO.output(self.D2,0)
        GPIO.output(self.D4,0)

        self.left_pwm1.ChangeDutyCycle(left_speed)
        self.right_pwm1.ChangeDutyCycle(right_speed)

    def reverse(self, left_speed, right_speed):
        self.left_pwm1.stop(0)
        self.right_pwm1.stop(0)
        GPIO.output(self.D1,0)
        GPIO.output(self.D3,0)

        self.left_pwm2.ChangeDutyCycle(left_speed)
        self.right_pwm2.ChangeDutyCycle(right_speed)

    def left(self, left_speed, right_speed):               # counter clockwise turn
        self.left_pwm1.stop(0)
        self.right_pwm1.stop(0)
        self.right_pwm2.stop(0)
        GPIO.output(self.D1,0)
        GPIO.output(self.D3,0)
        GPIO.output(self.D4,0)

        self.left_pwm2.ChangeDutyCycle(left_speed)

    def left_reverse(self, left_speed, right_speed):       # clockwise turn
        self.left_pwm2.stop(0)
        self.right_pwm1.stop(0)
        self.right_pwm2.stop(0)
        GPIO.output(self.D2,0)
        GPIO.output(self.D3,0)
        GPIO.output(self.D4,0)

        self.left_pwm1.ChangeDutyCycle(left_speed)

    def pivot_left(self, left_speed, right_speed):         # counter clockwise turn
        self.left_pwm1.stop(0)
        self.right_pwm2.stop(0)
        GPIO.output(self.D1,0)
        GPIO.output(self.D4,0)

        self.left_pwm2.ChangeDutyCycle(left_speed)
        self.right_pwm1.ChangeDutyCycle(right_speed)


    def right(self, left_speed, right_speed):              # clockwise turn
        self.left_pwm1.stop(0)
        self.left_pwm2.stop(0)
        self.right_pwm1.stop(0)
        GPIO.output(self.D1,0)
        GPIO.output(self.D2,0)
        GPIO.output(self.D3,0)

        self.right_pwm2.ChangeDutyCycle(right_speed)

    def right_reverse(self, left_speed, right_speed):      # counter clockwise turn
        self.left_pwm1.stop(0)
        self.left_pwm2.stop(0)
        self.right_pwm2.stop(0)
        GPIO.output(self.D1,0)
        GPIO.output(self.D2,0)
        GPIO.output(self.D4,0)

        self.right_pwm1.ChangeDutyCycle(right_speed)

    def pivot_right(self, left_speed, right_speed):        # clockwise turn
        self.left_pwm2.stop(0)
        self.right_pwm1.stop(0)
        GPIO.output(self.D2,0)
        GPIO.output(self.D3,0)

        self.left_pwm1.ChangeDutyCycle(left_speed)
        self.right_pwm2.ChangeDutyCycle(right_speed)



    def calculate_body_turn_radius(self, linear_speed, angular_speed):
        if angular_speed != 0.0:
            body_turn_radius = linear_speed / angular_speed
        else:
            # Not turning, infinite turn radius
            body_turn_radius = None
        return body_turn_radius

    def calculate_wheel_turn_radius(self, body_turn_radius, angular_speed, wheel):

        if body_turn_radius is not None:
            """
            if angular_speed > 0.0:
                angular_speed_sign = 1
            elif angular_speed < 0.0:
                angular_speed_sign = -1
            else:
                angular_speed_sign = 0.0
            """
            if wheel == "right":
                wheel_sign = 1
            elif wheel == "left":
                wheel_sign = -1
            else:
                assert False, "Wheel Name not supported, left or right only."

            wheel_turn_radius = body_turn_radius + ( wheel_sign * (self._wheel_distance / 2.0))
        else:
            wheel_turn_radius = None

        return wheel_turn_radius

    def calculate_wheel_rpm(self, linear_speed, angular_speed, wheel_turn_radius):
        """
        Omega_wheel = Linear_Speed_Wheel / Wheel_Radius
        Linear_Speed_Wheel = Omega_Turn_Body * Radius_Turn_Wheel
        --> If there is NO Omega_Turn_Body, Linear_Speed_Wheel = Linear_Speed_Body
        :param angular_speed:
        :param wheel_turn_radius:
        :return:
        """
        if wheel_turn_radius is not None:
            # The robot is turning
            wheel_rpm = (angular_speed * wheel_turn_radius) / self._wheel_radius
        else:
            # Its not turning therefore the wheel speed is the same as the body
            wheel_rpm = linear_speed / self._wheel_radius

        return wheel_rpm

    def set_wheel_movement(self, right_wheel_rpm, left_wheel_rpm):

        #print("W1,W2=["+str(right_wheel_rpm)+","+str(left_wheel_rpm)+"]")

        if right_wheel_rpm > 0.0 and left_wheel_rpm > 0.0:
            print("All forwards")

            if self.simple_mode:
                # We make it turn only on one wheel
                if right_wheel_rpm > left_wheel_rpm:
                    print("GO FORWARDS RIGHT")
                    self.right(right_wheel_rpm, left_wheel_rpm)
                if right_wheel_rpm < left_wheel_rpm:
                    print("GO FORWARDS LEFT")
                    self.left(right_wheel_rpm, left_wheel_rpm)
                if right_wheel_rpm == left_wheel_rpm:
                    print("GO FORWARDS")
                    self.forward(right_wheel_rpm, left_wheel_rpm)
            else:
                #print("GO FORWARDS")
                self.forward(right_wheel_rpm, left_wheel_rpm)


        elif right_wheel_rpm > 0.0 and left_wheel_rpm == 0.0:
            print("Right Wheel forwards, left stop")
            self.left(right_wheel_rpm, left_wheel_rpm)

        elif right_wheel_rpm > 0.0 and left_wheel_rpm < 0.0:
            print("Right Wheel forwards, left backwards --> Pivot left")
            self.pivot_left(right_wheel_rpm, left_wheel_rpm)
        elif right_wheel_rpm == 0.0 and left_wheel_rpm > 0.0:
            #print("Right stop, left forwards")
            self.right(right_wheel_rpm, left_wheel_rpm)

        elif right_wheel_rpm < 0.0 and left_wheel_rpm > 0.0:
            #print("Right backwards, left forwards --> Pivot right")
            self.pivot_right(right_wheel_rpm, left_wheel_rpm)
        elif right_wheel_rpm < 0.0 and left_wheel_rpm < 0.0:
            #print("All backwards")

            if self.simple_mode:
                # We make it turn only on one wheel
                if abs(right_wheel_rpm) > abs(left_wheel_rpm):
                    print("GO BACKWARDS RIGHT")
                    self.right_reverse(right_wheel_rpm, left_wheel_rpm)
                if abs(right_wheel_rpm) < abs(left_wheel_rpm):
                    print("GO BACKWARDS LEFT")
                    self.left_reverse(right_wheel_rpm, left_wheel_rpm)
                if right_wheel_rpm == left_wheel_rpm:
                    print("GO BACKWARDS")
                    self.reverse(right_wheel_rpm, left_wheel_rpm)
            else:
                self.reverse(right_wheel_rpm, left_wheel_rpm)


        elif right_wheel_rpm == 0.0 and left_wheel_rpm == 0.0:
            print("Right stop, left stop")
            self.stop()
        else:
            assert False, "A case wasn't considered==>"+str(right_wheel_rpm)+","+str(left_wheel_rpm)
            pass

    def set_cmd_vel(self, linear_speed, angular_speed):

        body_turn_radius = self.calculate_body_turn_radius(linear_speed, angular_speed)

        wheel = "right"
        right_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                   angular_speed,
                                                                   wheel)

        wheel = "left"
        left_wheel_turn_radius = self.calculate_wheel_turn_radius(body_turn_radius,
                                                                  angular_speed,
                                                                  wheel)

        right_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, right_wheel_turn_radius)
        left_wheel_rpm = self.calculate_wheel_rpm(linear_speed, angular_speed, left_wheel_turn_radius)


        self.set_wheel_movement(right_wheel_rpm, left_wheel_rpm)
