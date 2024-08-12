import RPi.GPIO as GPIO
import time

# Disable GPIO warnings
GPIO.setwarnings(False)

# GPIO pin setup
PUL = 17  # Pulse pin
DIR = 27  # Direction pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

# Set direction
GPIO.output(DIR, GPIO.HIGH)  # Change to GPIO.LOW for opposite direction
print("Direction set to HIGH")

# Number of steps
steps = 1600

try:
    for i in range(steps):
        GPIO.output(PUL, GPIO.HIGH)
        print(f"Step {i+1}: PUL HIGH")
        time.sleep(0.01)  # Slower pulse for easier observation
        GPIO.output(PUL, GPIO.LOW)
        print(f"Step {i+1}: PUL LOW")
        time.sleep(0.01)
except KeyboardInterrupt:
    print("Script interrupted by user")
finally:
    GPIO.cleanup()
    print("GPIO cleanup complete")
