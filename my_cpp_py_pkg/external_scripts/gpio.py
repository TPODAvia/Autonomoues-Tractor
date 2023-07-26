import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18, 100)
pwm.start(50)

try:
    while True:
        for dc in range(0, 101, 5):
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.1)
        for dc in range(100, -1, -5):
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.1)
except KeyboardInterrupt:
    pass

pwm.stop()
GPIO.cleanup()