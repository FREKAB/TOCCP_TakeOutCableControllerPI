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
STOP_BUTTON = 25  # Stop button
EMERGENCY_STOP = 24  # Emergency stop button

steps_per_rotation = 1600  # Adjust based on your motor and stepper driver
last_button_press = 0
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

def reset_motor_driver():
    print("Resetting motor driver...")
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    print("Motor driver reset complete.")

def run_motor(direction, speed=0.001):
    GPIO.output(DIR, direction)
    GPIO.output(PUL, GPIO.LOW)
    time.sleep(speed)
    GPIO.output(PUL, GPIO.HIGH)
    time.sleep(speed)
    
def stop_motor():
    global motor_running
    motor_running = False
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Disable the motor
    print("Motor stopped")


def emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Emergency brake activated!")

def release_emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Re-enable the motor
    print("Emergency brake released")

def check_buttons():
    global motor_running
    max_speed = 0.0002  # Minimum delay between pulses (maximum speed)
    start_speed = 0.001  # Starting speed (larger delay)
    accel_steps = 1600   # Number of steps for acceleration

    while True:
        if GPIO.input(FWD_BUTTON) == GPIO.LOW and not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            GPIO.output(DIR, GPIO.LOW)  # Set direction to forward
            
            # Acceleration phase
            for i in range(accel_steps):
                if GPIO.input(FWD_BUTTON) == GPIO.HIGH:
                    break
                current_speed = start_speed - (start_speed - max_speed) * (i / accel_steps)
                GPIO.output(PUL, GPIO.HIGH)
                time.sleep(current_speed)
                GPIO.output(PUL, GPIO.LOW)
                time.sleep(current_speed)
                
                if GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                    emergency_brake()
                    while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                        time.sleep(0.01)
                    release_emergency_brake()
                    break

            # Constant speed phase
            while GPIO.input(FWD_BUTTON) == GPIO.LOW:
                if GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                    emergency_brake()
                    while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                        time.sleep(0.01)
                    release_emergency_brake()
                else:
                    GPIO.output(PUL, GPIO.HIGH)
                    time.sleep(max_speed)
                    GPIO.output(PUL, GPIO.LOW)
                    time.sleep(max_speed)

            GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor when button is released
            print("Forward button released")

        elif GPIO.input(BWD_BUTTON) == GPIO.LOW and not motor_running:
            GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
            GPIO.output(DIR, GPIO.HIGH)  # Set direction to backward
            
            # Acceleration phase
            for i in range(accel_steps):
                if GPIO.input(BWD_BUTTON) == GPIO.HIGH:
                    break
                current_speed = start_speed - (start_speed - max_speed) * (i / accel_steps)
                GPIO.output(PUL, GPIO.HIGH)
                time.sleep(current_speed)
                GPIO.output(PUL, GPIO.LOW)
                time.sleep(current_speed)
                
                if GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                    emergency_brake()
                    while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                        time.sleep(0.01)
                    release_emergency_brake()
                    break

            # Constant speed phase
            while GPIO.input(BWD_BUTTON) == GPIO.LOW:
                if GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                    emergency_brake()
                    while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                        time.sleep(0.01)
                    release_emergency_brake()
                else:
                    GPIO.output(PUL, GPIO.HIGH)
                    time.sleep(max_speed)
                    GPIO.output(PUL, GPIO.LOW)
                    time.sleep(max_speed)

            GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor when button is released
            print("Backward button released")

        elif GPIO.input(STOP_BUTTON) == GPIO.LOW:
            stop_motor()
        elif GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
            emergency_brake()
            while GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
                time.sleep(0.01)
            release_emergency_brake()
        time.sleep(0.01)  # Small delay to prevent excessive CPU usage

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")


# Global variable to store the last time we received a "run manual" message
last_manual_run_time = 0
timeout_threshold = 0.1  # Timeout threshold in seconds to stop the motor if no "run manual" is received

def on_message(client, userdata, msg):
    global motor_running
    if msg.topic == "motor/control":
        try:
            command = msg.payload.decode().strip().lower()
            if command == "slowdown":
                print("MQTT command: slowdown")
                motor_speed = 0.005  # Increase sleep time to slow down the motor
                motor_running = True

            elif command == "stop":
                print("MQTT command: stop")
                stop_motor()

            else:
                # Default case: run the motor normally
                rotations = float(command)
                steps = int(abs(rotations) * steps_per_rotation)
                if steps > 0:
                    direction = GPIO.HIGH if rotations > 0 else GPIO.LOW
                    print(f"MQTT command: {'forward' if rotations > 0 else 'backward'} for {steps} steps")
                    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
                    GPIO.output(DIR, direction)
                    motor_speed = 0.001  # Normal speed
                    motor_running = True

                    for step in range(steps):
                        if not motor_running:
                            break  # Stop the motor if a stop command has been received
                        run_motor(direction, motor_speed)

                    motor_running = False
                    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor after movement

        except ValueError:
            print(f"Unknown command: {msg.payload.decode()}")


# Motor control loop to check for "run manual" heartbeat
def motor_control_loop():
    global motor_running, last_manual_run_time
    direction = GPIO.LOW  # Example motor direction (forward)
    speed = 0.001  # Motor step speed (adjust as necessary)

    while True:
        if motor_running:
            # Run the motor for one step
            run_motor(direction, speed)

            # Check if we have timed out (i.e., no "run manual" received within the threshold)
            if time.time() - last_manual_run_time > timeout_threshold:
                print("Run manual command not received in time, stopping motor")
                stop_motor()

        time.sleep(0.001)  # Small delay to avoid high CPU usage




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