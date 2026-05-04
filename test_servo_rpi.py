import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.OUT)
pwm = GPIO.PWM(19, 50)
pwm.start(7.5)
print('Center')
time.sleep(2)
pwm.ChangeDutyCycle(5.0)
print('Left')
time.sleep(2)
pwm.ChangeDutyCycle(10.0)
print('Right')
time.sleep(2)
pwm.stop()
GPIO.cleanup()
