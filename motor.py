import time
import RPi.GPIO as GPIO 
import threading
from getkey import getkey, keys

class motor:
    def __init__(self,first_coil,second_coil):
        GPIO.setmode(GPIO.BCM)
        self.first_coil = first_coil
        self.second_coil = second_coil
        self.speed = 50
        self.frequency = 5000
      
        GPIO.setup(self.first_coil,GPIO.OUT)
        GPIO.setup(self.second_coil,GPIO.OUT)
       
        GPIO.output(self.first_coil,0)
        GPIO.output(self.second_coil,0)
              
        self.first_coil_pwm = GPIO.PWM(self.first_coil,self.frequency)
        self.second_coil_pwm = GPIO.PWM(self.second_coil,self.frequency)
        
    def motor_break(self):
        self.first_coil_pwm.stop(0)
        self.second_coil_pwm.stop(0)
        GPIO.output(self.first_coil,0)
        GPIO.output(self.second_coil,0)
        
    def reverse(self):
        self.motor_break()
        self.first_coil_pwm.start(self.speed)
           
    def forward(self):
        self.motor_break()
        self.second_coil_pwm.start(self.speed)

    def set_speed(self,speed):
        self.speed = speed
        
    def set_frequency(self,frequency):
        self.frequency = frequency

