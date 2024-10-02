import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import threading

# GPIO pin setup
PUL = 17
DIR = 27
ENABLE_PIN = 12
FWD_BUTTON = 22
BWD_BUTTON = 23
STOP_BUTTON = 25
EMERGENCY_STOP = 24

steps_per_rotation = 1600
debounce_time = 20  # millisecond

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
emergency_stop = False
motor_running = False
motor_speed = 0.001  # Default motor speed
last_manual_run_time = 0
timeout_threshold = 1  # Timeout threshold in seconds to stop motor if no "run manual" received

# Motor control functions
def run_motor(direction, speed=0.001):
    GPIO.output(DIR, direction)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(speed)
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(speed)

def stop_motor():
    global motor_running
    motor_running = False
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Motor stopped")

def emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Emergency brake activated!")

def release_emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Re-enable the motor
    print("Emergency brake released")

# Function to handle buttons
def check_buttons():
    global motor_running
    max_speed = 0.0002
    start_speed = 0.001
    accel_steps = 1600

    while True:
        if GPIO.input(FWD_BUTTON) == GPIO.LOW and not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)
            GPIO.output(DIR, GPIO.LOW)
            for i in range(accel_steps):
                if GPIO.input(FWD_BUTTON) == GPIO.HIGH:
                    break
                current_speed = start_speed - (start_speed - max_speed) * (i / accel_steps)
                run_motor(GPIO.LOW, current_speed)
            motor_running = True
        elif GPIO.input(BWD_BUTTON) == GPIO.LOW and not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)
            GPIO.output(DIR, GPIO.HIGH)
            for i in range(accel_steps):
                if GPIO.input(BWD_BUTTON) == GPIO.HIGH:
                    break
                current_speed = start_speed - (start_speed - max_speed) * (i / accel_steps)
                run_motor(GPIO.HIGH, current_speed)
            motor_running = True
        elif GPIO.input(STOP_BUTTON) == GPIO.LOW:
            stop_motor()
        elif GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
            emergency_brake()
            while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                time.sleep(0.01)
            release_emergency_brake()

        time.sleep(0.01)

# MQTT callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    global motor_running, last_manual_run_time, motor_speed
    command = msg.payload.decode().strip().lower()

    if command == "run manual":
        last_manual_run_time = time.time()
        if not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)
            motor_running = True
            motor_speed = 0.001
            print("Motor started manually")

    elif command == "slowdown":
        motor_speed = 0.005
        print("MQTT command: slowdown")

    elif command == "stop":
        stop_motor()

    else:
        try:
            rotations = float(command)
            steps = int(abs(rotations) * steps_per_rotation)
            if steps > 0:
                direction = GPIO.LOW if rotations > 0 else GPIO.HIGH
                GPIO.output(ENABLE_PIN, GPIO.LOW)
                motor_speed = 0.001  # Default speed for rotation command
                motor_running = True
                for step in range(steps):
                    if not motor_running:
                        break
                    run_motor(direction)
                motor_running = False
                GPIO.output(ENABLE_PIN, GPIO.HIGH)
        except ValueError:
            print(f"Unknown command: {command}")

# Motor control loop to monitor "run manual" heartbeat
def motor_control_loop():
    global motor_running, last_manual_run_time, motor_speed
    while True:
        if motor_running:
            # Keep the motor running
            run_motor(GPIO.LOW, motor_speed)

            # Check if we timed out (i.e., no "run manual" received within timeout_threshold)
            if time.time() - last_manual_run_time > timeout_threshold:
                print("Timeout, stopping motor")
                stop_motor()

        time.sleep(0.001)  # Small delay to avoid high CPU usage

# MQTT and motor control setup
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PUL, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)
    GPIO.setup(ENABLE_PIN, GPIO.OUT)
    GPIO.output(ENABLE_PIN, GPIO.HIGH)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    mqtt_broker_ip = "192.168.10.9"
    client.connect(mqtt_broker_ip, 1883, 60)

    return client

# Start the system
def start():
    client = setup()

    # Start button checking in a separate thread
    button_thread = threading.Thread(target=check_buttons)
    button_thread.daemon = True
    button_thread.start()

    # Start motor control in a separate thread
    motor_thread = threading.Thread(target=motor_control_loop)
    motor_thread.daemon = True
    motor_thread.start()

    client.loop_forever()

# Reset motor driver and start the system
try:
    reset_motor_driver()
    start()
except KeyboardInterrupt:
    print("Program interrupted by user")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    GPIO.cleanup()
    print("GPIO cleanup done.")
