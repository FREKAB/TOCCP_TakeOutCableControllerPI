import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
PIN_TO_TEST = 22

GPIO.setup(PIN_TO_TEST, GPIO.IN, pull_up_down=GPIO.PUD_UP)

try:
    print(f"Reading GPIO {PIN_TO_TEST}. Press Ctrl+C to exit.")
    while True:
        state = GPIO.input(PIN_TO_TEST)
        print(f"GPIO {PIN_TO_TEST} state: {state}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Script terminated by user")
finally:
    GPIO.cleanup()
    print("GPIO cleanup done")