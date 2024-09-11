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
    GPIO.output(DIR, direction)
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(speed)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(speed)

def stop_motor():
    print("Motor stopped")

def emergency_brake():
    global emergency_stop
    emergency_stop = True
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Emergency brake activated!")

def check_buttons():
    global emergency_stop
    while not emergency_stop:
        if GPIO.input(FWD_BUTTON) == GPIO.LOW:
            print("Running motor forward")
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            while GPIO.input(FWD_BUTTON) == GPIO.LOW and not emergency_stop:
                run_motor(GPIO.HIGH)
            GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor when button is released
            stop_motor()
        elif GPIO.input(BWD_BUTTON) == GPIO.LOW:
            print("Running motor backward")
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            while GPIO.input(BWD_BUTTON) == GPIO.LOW and not emergency_stop:
                run_motor(GPIO.LOW)
            GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor when button is released
            stop_motor()
        elif GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
            emergency_brake()
        time.sleep(0.01)  # Small delay to prevent excessive CPU usage

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    if msg.topic == "motor/control":
        command = msg.payload.decode().strip().lower()
        if command == "forward":
            print("MQTT command: forward")
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            run_motor(GPIO.HIGH)
            GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor after one step
        elif command == "backward":
            print("MQTT command: backward")
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            run_motor(GPIO.LOW)
            GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor after one step
        elif command == "stop":
            stop_motor()
        else:
            print(f"Unknown command: {command}")

try:
    reset_motor_driver()  # Reset the motor driver at the start

    # Start button checking in a separate thread
    button_thread = threading.Thread(target=check_buttons)
    button_thread.daemon = True
    button_thread.start()

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
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Ensure motor is disabled on program exit
    GPIO.cleanup()
    print("GPIO cleanup done.")