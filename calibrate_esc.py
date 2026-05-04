import lgpio
import time

ESC_PIN  = 18
PWM_FREQ = 50

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, ESC_PIN, 0)

def set_duty(duty):
    lgpio.tx_pwm(h, ESC_PIN, PWM_FREQ, duty)

print("Step 1: Setting FULL throttle (10%) — plug in battery NOW if not already")
set_duty(10.0)
time.sleep(3)

print("Step 2: Setting NEUTRAL (7.5%) — ESC should beep to confirm calibration")
set_duty(7.5)
time.sleep(3)

print("Step 3: Testing forward at 8.5%...")
set_duty(8.5)
time.sleep(3)

print("Step 4: Testing forward at 9.0%...")
set_duty(9.0)
time.sleep(3)

print("Back to neutral...")
set_duty(7.5)
time.sleep(1)

lgpio.tx_pwm(h, ESC_PIN, 0, 0)
lgpio.gpiochip_close(h)
print("Done.")
