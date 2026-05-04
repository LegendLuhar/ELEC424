import lgpio
import time

SERVO_PIN = 19
PWM_FREQ  = 50

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, SERVO_PIN, 0)

def set_duty(duty):
    lgpio.tx_pwm(h, SERVO_PIN, PWM_FREQ, duty)
    print(f"  duty={duty}%")

print("Center (7.5%)...")
set_duty(7.5)
time.sleep(2)

print("Full LEFT (5.0%)...")
set_duty(5.0)
time.sleep(2)

print("Center (7.5%)...")
set_duty(7.5)
time.sleep(2)

print("Full RIGHT (10.0%)...")
set_duty(10.0)
time.sleep(2)

print("Center (7.5%)...")
set_duty(7.5)
time.sleep(2)

lgpio.tx_pwm(h, SERVO_PIN, 0, 0)
lgpio.gpiochip_close(h)
print("Done.")
