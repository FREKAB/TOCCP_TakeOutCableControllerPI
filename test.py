import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
PIN_TO_TEST = 22

GPIO.setup(PIN_TO_TEST, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def callback(channel):
    print(f"Event detected on channel {channel}")

try:
    GPIO.add_event_detect(PIN_TO_TEST, GPIO.FALLING, callback=callback, bouncetime=300)
    print(f"Event detection set up for GPIO {PIN_TO_TEST}")
    while True:
        time.sleep(1)
except Exception as e:
    print(f"Error: {e}")
finally:
    GPIO.cleanup()
    print("GPIO cleanup done")