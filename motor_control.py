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
motor_running = False

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.setup(FWD_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BWD_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(STOP_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(EMERGENCY_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global variables for motor state
stop_signal_received = False

def reset_motor_driver():
    print("Resetting motor driver...")
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(ENABLE_PIN, GPIO.HIGH)
    print("Motor driver reset complete.")

def run_motor(direction, speed=0.001, steps=None):
    global stop_signal_received
    GPIO.output(DIR, direction)
    stop_signal_received = False
    step_count = 0
    
    while steps is None or step_count < steps:
        if stop_signal_received:
            break
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(speed)
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(speed)
        step_count += 1

    GPIO.output(ENABLE_PIN, GPIO.LOW)
    print(f"Motor run completed. Steps executed: {step_count}")

def stop_motor():
    global stop_signal_received, motor_running
    stop_signal_received = True
    motor_running = False
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Disable the motor
    print("Motor stopped.")

def emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Disable the motor
    print("Emergency brake activated!")

def release_emergency_brake():
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Re-enable the motor
    print("Emergency brake released")

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code {rc}")
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    global motor_running
    if msg.topic == "motor/control":
        command = msg.payload.decode().strip().lower()
        print(f"MQTT message received: {command}")
        
        if command.startswith("run"):
            if not motor_running:
                motor_running = True
                try:
                    # Extract duration from command and convert to float
                    duration = float(command.split()[1])
                    print(f"Running motor for {duration} seconds.")
                    
                    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the motor
                    direction = GPIO.LOW  # Adjust direction as needed (forward)
                    thread = threading.Thread(target=run_motor, args=(direction, 0.001, steps_per_rotation * duration))
                    thread.start()
                except Exception as e:
                    print(f"Invalid run command: {e}")
            else:
                print("Motor is already running.")
        
        elif command == "stop":
            print("Stopping motor via MQTT")
            stop_motor()

try:
    reset_motor_driver()  # Reset the motor driver at the start

    # MQTT setup
    mqtt_broker_ip = "192.168.10.9"  # Replace with your broker IP
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(mqtt_broker_ip, 1883, 60)
    client.loop_start()  # Run the MQTT loop in a background thread

    # Main program loop (for example, monitoring buttons)
    while True:
        if GPIO.input(STOP_BUTTON) == GPIO.LOW:
            stop_motor()
        elif GPIO.input(EMERGENCY_STOP) == GPIO.LOW:
            emergency_brake()
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Program interrupted by user.")
finally:
    GPIO.output(ENABLE_PIN, GPIO.HIGH)  # Ensure motor is disabled on program exit
    GPIO.cleanup()
    print("GPIO cleanup done.")
