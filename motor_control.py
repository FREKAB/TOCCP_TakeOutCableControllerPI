import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import threading

try:
    # GPIO setup
    PUL = 17
    DIR = 27
    steps_per_rotation = 1600  # Adjust based on your motor and stepper driver

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PUL, GPIO.OUT)
    GPIO.setup(DIR, GPIO.OUT)

    def run_motor(rotations):
        try:
            rotations = float(rotations.strip())
            total_steps = int(rotations * steps_per_rotation)
            GPIO.output(DIR, GPIO.HIGH)  # Set direction

            # Acceleration and deceleration parameters
            accel_steps = min(400, total_steps // 4)  # Accelerate for 400 steps or 1/4 of total, whichever is smaller
            decel_steps = accel_steps
            const_speed_steps = total_steps - accel_steps - decel_steps

            # Acceleration
            for i in range(accel_steps):
                delay = 0.001 - (0.0009 * i / accel_steps)  # Start slow, gradually speed up
                GPIO.output(PUL, GPIO.HIGH)
                time.sleep(delay)
                GPIO.output(PUL, GPIO.LOW)
                time.sleep(delay)

            # Constant speed
            for i in range(const_speed_steps):
                GPIO.output(PUL, GPIO.HIGH)
                time.sleep(0.0001)
                GPIO.output(PUL, GPIO.LOW)
                time.sleep(0.0001)

            # Deceleration
            for i in range(decel_steps):
                delay = 0.0001 + (0.0009 * i / decel_steps)  # Start fast, gradually slow down
                GPIO.output(PUL, GPIO.HIGH)
                time.sleep(delay)
                GPIO.output(PUL, GPIO.LOW)
                time.sleep(delay)

            print(f"Motor run complete: {rotations} rotations")
        except ValueError as e:
            print(f"Error: {e}. Received invalid rotations value: {rotations}")



    def on_connect(client, userdata, flags, rc):
        print("Connected with result code " + str(rc))
        client.subscribe("motor/control")

    def on_message(client, userdata, msg):
        if msg.topic == "motor/control":
            rotations = msg.payload.decode()
            print(f"RUN command received for {rotations} rotations")
            # Run the motor in a separate thread to avoid blocking
            threading.Thread(target=run_motor, args=(rotations,)).start()

    # MQTT setup
    mqtt_broker_ip = "192.168.10.9"  # Replace with your broker IP
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(mqtt_broker_ip, 1883, 60)
    client.loop_forever()  # Keep listening for incoming messages

    # Your existing code
except Exception as e:
    print(f"Exception occurred: {e}")
    raise
