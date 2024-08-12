import time
import RPi.GPIO as GPIO

# GPIO pin setup for the first motor according to the HAT
DIR_PIN = 13   # Direction pin
STEP_PIN = 19  # Step pin
ENABLE_PIN = 12  # Enable pin

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

# Function to move the motor
def move_motor(steps, direction, delay):
    # Enable the motor
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    
    # Set the direction
    GPIO.output(DIR_PIN, direction)
    
    # Move the motor
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(delay)
    
    # Disable the motor
    GPIO.output(ENABLE_PIN, GPIO.HIGH)

try:

 for _ in range(5):  # Repeat the movement 5 times
    move_motor(steps=10000, direction=GPIO.HIGH, delay=0.005)
    time.sleep(1)
    move_motor(steps=10000, direction=GPIO.LOW, delay=0.005)
    time.sleep(1)


except KeyboardInterrupt:
    print("Program interrupted!")

finally:
    GPIO.cleanup()