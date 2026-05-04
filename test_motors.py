import lgpio
import time

ESC_PIN   = 18
SERVO_PIN = 19
PWM_FREQ  = 50

h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, ESC_PIN,   0)
lgpio.gpio_claim_output(h, SERVO_PIN, 0)

def set_duty(pin, duty):
    lgpio.tx_pwm(h, pin, PWM_FREQ, duty)

print("Setting neutral on both (7.5%)...")
set_duty(ESC_PIN,   7.5)
set_duty(SERVO_PIN, 7.5)
time.sleep(3)

print("Testing SERVO — turning LEFT (6.5%)...")
set_duty(SERVO_PIN, 6.5)
time.sleep(2)

print("Testing SERVO — turning RIGHT (8.5%)...")
set_duty(SERVO_PIN, 8.5)
time.sleep(2)

print("Testing SERVO — back to center...")
set_duty(SERVO_PIN, 7.5)
time.sleep(1)

print("Testing ESC — slow forward (8.0%)...")
set_duty(ESC_PIN, 8.0)
time.sleep(3)

print("Back to neutral...")
set_duty(ESC_PIN,   7.5)
set_duty(SERVO_PIN, 7.5)
time.sleep(1)

lgpio.tx_pwm(h, ESC_PIN,   0, 0)
lgpio.tx_pwm(h, SERVO_PIN, 0, 0)
lgpio.gpiochip_close(h)
print("Done.")
