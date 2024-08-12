import time
import RPi.GPIO as GPIO

DIR_PIN = 13   # Direction pin
STEP_PIN = 19  # Step pin
ENABLE_PIN = 12  # Enable pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

GPIO.output(ENABLE_PIN, GPIO.LOW)

# Test forward direction
GPIO.output(DIR_PIN, GPIO.HIGH)
for _ in range(1000):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(0.005)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(0.005)

time.sleep(1)  # Wait for 1 second

# Test reverse direction
GPIO.output(DIR_PIN, GPIO.LOW)
for _ in range(1000):
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(0.005)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(0.005)

GPIO.cleanup()
