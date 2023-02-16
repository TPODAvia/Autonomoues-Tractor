import time
import RPi.GPIO as GPIO 
import threading
from getkey import getkey, keys
SPEED = 80 #0 100

class motor_1:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.GPIO_motor_1_12 = 12
        self.GPIO_motor_1_18 = 18
      
        GPIO.setup(self.GPIO_motor_1_12,GPIO.OUT)
        GPIO.setup(self.GPIO_motor_1_18,GPIO.OUT)
       
        GPIO.output(self.GPIO_motor_1_12,0)
        GPIO.output(self.GPIO_motor_1_18,0)
              
        self.pwm_motor_1_12 = GPIO.PWM(self.GPIO_motor_1_12,5000)
        self.pwm_motor_1_18 = GPIO.PWM(self.GPIO_motor_1_18,5000)
        
    def motor_break(self):
        self.pwm_motor_1_12.stop(0)
        self.pwm_motor_1_18.stop(0)
        
    def reverse(self):
        #self.motor_break()
        self.pwm_motor_1_12.start(SPEED)
           
    def forward(self):
        self.motor_break()
        self.pwm_motor_1_18.start(SPEED) 
    
    def turn_right(self):
        #self.motor_break()
        self.pwm_motor_1_12.start(40)

class motor_2:
    def __init__(self):
        
        GPIO.setmode(GPIO.BCM)
        self.GPIO_motor_2_19 = 19
        self.GPIO_motor_2_13 = 13   
    
        GPIO.setup(self.GPIO_motor_2_19,GPIO.OUT)
        GPIO.setup(self.GPIO_motor_2_13,GPIO.OUT)
    
        GPIO.output(self.GPIO_motor_2_19,0)
        GPIO.output(self.GPIO_motor_2_13,0)  
              
        self.pwm_motor_2_19 = GPIO.PWM(self.GPIO_motor_2_19,5000)  
        self.pwm_motor_2_13 = GPIO.PWM(self.GPIO_motor_2_13,5000)
     
    def motor_break(self):
        self.pwm_motor_2_19.stop(0)
        self.pwm_motor_2_13.stop(0)
        
    def reverse(self):
        #self.motor_break()
        self.pwm_motor_2_19.start(SPEED)     
       
    def forward(self):
        self.motor_break()
        self.pwm_motor_2_13.start(SPEED) 
        
    


m1=motor_1()
m2=motor_2()

def break_both():
    m1.motor_break()  
    m2.motor_break()  
def turn_right():
    break_both()
    m1.forward()
def turn_left():
    break_both()
    m2.forward()
while 1:
    btn = getkey()
    if btn == "q":
        break
    if btn == "w":
        break_both()
        m1.forward()
        m2.forward()
    if btn == "s":
        break_both()
        m1.reverse()
        m2.reverse()
    if btn == "b":
        break_both()
    if btn == "a":
        turn_left()
    if btn == "d":
        turn_right()
        

GPIO.cleanup()

