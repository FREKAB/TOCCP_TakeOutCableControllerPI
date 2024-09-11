import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import threading

# GPIO pin setup
PUL = 17
DIR = 27
ENABLE_PIN = 12  # Enable pin for the motor driver
FWD_BUTTON = 22  # Forward button
BWD_BUTTON = 23  # Backward button
STOP_BUTTON = 24  # Stop button
EMERGENCY_STOP = 25  # Emergency stop button

steps_per_rotation = 1600  # Adjust based on your motor and stepper driver

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.setup(FWD_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BWD_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(STOP_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(EMERGENCY_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global variables
running = False
emergency_stop = False

def reset_motor_driver():
    print("Resetting motor driver...")
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    print("Motor driver reset complete.")

def run_motor(direction, speed=0.0001):
    global running, emergency_stop
    running = True
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
    GPIO.output(DIR, direction)
    
    while running and not emergency_stop:
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(speed)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(speed)
    
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    running = False

def run_motor_forward():
    print("Running motor forward")
    run_motor(GPIO.HIGH)

def run_motor_backward():
    print("Running motor backward")
    run_motor(GPIO.LOW)

def stop_motor():
    global running
    running = False
    print("Stopping motor")

def emergency_brake():
    global emergency_stop
    emergency_stop = True
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Immediately disable the motor
    print("Emergency brake activated!")

def button_callback(channel):
    if channel == FWD_BUTTON and not running:
        threading.Thread(target=run_motor_forward).start()
    elif channel == BWD_BUTTON and not running:
        threading.Thread(target=run_motor_backward).start()
    elif channel == STOP_BUTTON:
        stop_motor()
    elif channel == EMERGENCY_STOP:
        emergency_brake()

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    if msg.topic == "motor/control":
        command = msg.payload.decode().strip().lower()
        if command == "forward":
            threading.Thread(target=run_motor_forward).start()
        elif command == "backward":
            threading.Thread(target=run_motor_backward).start()
        elif command == "stop":
            stop_motor()
        else:
            print(f"Unknown command: {command}")

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PUL, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(ENABLE_PIN, GPIO.OUT)
    GPIO.setup(FWD_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BWD_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(STOP_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(EMERGENCY_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def setup_event_detection():
    try:
        GPIO.add_event_detect(FWD_BUTTON, GPIO.FALLING, callback=button_callback, bouncetime=300)
        GPIO.add_event_detect(BWD_BUTTON, GPIO.FALLING, callback=button_callback, bouncetime=300)
        GPIO.add_event_detect(STOP_BUTTON, GPIO.FALLING, callback=button_callback, bouncetime=300)
        GPIO.add_event_detect(EMERGENCY_STOP, GPIO.FALLING, callback=button_callback, bouncetime=300)
    except RuntimeError as e:
        print(f"Error setting up event detection: {e}")
        print("This might be due to GPIO pins already being in use.")
        return False
    return True

try:
    setup_gpio()
    if not setup_event_detection():
        raise Exception("Failed to set up GPIO event detection")

    reset_motor_driver()  # Reset the motor driver at the start

    # MQTT setup
    mqtt_broker_ip = "192.168.10.9"  # Replace with your broker IP
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(mqtt_broker_ip, 1883, 60)
    client.loop_forever()  # Keep listening for incoming messages

except KeyboardInterrupt:
    print("Program interrupted by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Ensure motor is disabled
    GPIO.cleanup()
    print("GPIO cleanup done.")
