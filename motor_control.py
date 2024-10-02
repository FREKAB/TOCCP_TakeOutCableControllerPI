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
manual_mode = False  # To track if the motor is running in manual mode
last_manual_run_time = 0
timeout_threshold = 1  # Timeout threshold for manual run in seconds

# Motor control functions
def run_motor(direction, speed=0.001):
    GPIO.output(DIR, direction)
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(speed)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(speed)

def stop_motor():
    global motor_running, manual_mode
    motor_running = False
    manual_mode = False  # Stop manual mode if it was running
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Motor stopped")

def emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Emergency brake activated!")

def release_emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Re-enable the motor
    print("Emergency brake released")

# Reset motor driver function
def reset_motor_driver():
    print("Resetting motor driver...")
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    print("Motor driver reset complete.")

# Button handling logic
def check_buttons():
    global motor_running
    max_speed = 0.0002
    start_speed = 0.001
    accel_steps = 1600

    while True:
        if GPIO.input(FWD_BUTTON) == GPIO.LOW and not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            GPIO.output(DIR, GPIO.HIGH)  # Set direction to forward
            motor_running = True
            current_speed = start_speed

            # Acceleration phase
            for i in range(accel_steps):
                if GPIO.input(FWD_BUTTON) == GPIO.HIGH:
                    break
                current_speed = start_speed - (start_speed - max_speed) * (i / accel_steps)
                run_motor(GPIO.HIGH, current_speed)

            # Constant speed phase while the button is pressed
            while GPIO.input(FWD_BUTTON) == GPIO.LOW:
                run_motor(GPIO.HIGH, max_speed)
            
            # Stop the motor when the button is released
            stop_motor()
            print("Forward button released")

        elif GPIO.input(BWD_BUTTON) == GPIO.LOW and not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            GPIO.output(DIR, GPIO.HIGH)  # Set direction to backward
            motor_running = True
            current_speed = start_speed

            # Acceleration phase
            for i in range(accel_steps):
                if GPIO.input(BWD_BUTTON) == GPIO.HIGH:
                    break
                current_speed = start_speed - (start_speed - max_speed) * (i / accel_steps)
                run_motor(GPIO.HIGH, current_speed)

            # Constant speed phase while the button is pressed
            while GPIO.input(BWD_BUTTON) == GPIO.LOW:
                run_motor(GPIO.HIGH, max_speed)

            # Stop the motor when the button is released
            stop_motor()
            print("Backward button released")

        elif GPIO.input(STOP_BUTTON) == GPIO.LOW:
            stop_motor()

        elif GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
            emergency_brake()
            while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                time.sleep(0.01)  # Wait for the emergency stop to be released
            release_emergency_brake()

        time.sleep(0.01)  # Small delay to prevent excessive CPU usage


# MQTT callback functions
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    global motor_running, last_manual_run_time, motor_speed, manual_mode
    command = msg.payload.decode().strip().lower()

    if command == "run manual":
        last_manual_run_time = time.time()
        manual_mode = True  # Mark motor as in manual mode

        if not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)
            motor_running = True
            motor_speed = 0.001
            print("Motor started manually")

    elif command == "slowdown":
        motor_speed = 0.005  # Increase sleep time to slow down the motor
        print("MQTT command: slowdown")

    elif command == "stop":
        stop_motor()

    else:
        # Handle rotation command (normal operation)
        try:
            rotations = float(command)
            steps = int(abs(rotations) * steps_per_rotation)  # Convert rotations to steps
            if steps > 0:
                direction = GPIO.LOW if rotations > 0 else GPIO.HIGH
                print(f"MQTT command: {'forward' if rotations > 0 else 'backward'} for {steps} steps")
                GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
                motor_speed = 0.001  # Default speed for rotation command
                motor_running = True
                manual_mode = False  # Not manual mode for this type of run

                for step in range(steps):
                    if not motor_running:
                        break  # Stop the motor if a stop command has been received
                    run_motor(direction, motor_speed)

                motor_running = False
                GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor after movement
        except ValueError:
            print(f"Unknown command: {command}")


# Motor control loop to monitor "run manual" heartbeat
def motor_control_loop():
    global motor_running, last_manual_run_time, motor_speed, manual_mode
    while True:
        if motor_running:
            run_motor(GPIO.LOW, motor_speed)

            # Check timeout only if in manual mode
            if manual_mode and time.time() - last_manual_run_time > timeout_threshold:
                print("Timeout, stopping motor")
                stop_motor()

        time.sleep(0.001)


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
