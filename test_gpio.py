import lgpio
import time

h = lgpio.gpiochip_open(4)
lgpio.gpio_claim_output(h, 18, 0)

print("GPIO18 HIGH for 2 seconds...")
lgpio.gpio_write(h, 18, 1)
time.sleep(2)

print("GPIO18 LOW for 2 seconds...")
lgpio.gpio_write(h, 18, 0)
time.sleep(2)

print("GPIO18 HIGH again...")
lgpio.gpio_write(h, 18, 1)
time.sleep(2)

lgpio.gpio_write(h, 18, 0)
lgpio.gpiochip_close(h)
print("Done.")
