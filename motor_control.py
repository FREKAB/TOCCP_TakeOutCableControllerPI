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
            # Sanitize input by stripping any extra spaces and ensuring it's a float
            rotations = float(rotations.strip())
            total_steps = int(rotations * steps_per_rotation)  # Convert rotations to steps
            GPIO.output(DIR, GPIO.HIGH)  # Set direction
            for i in range(total_steps):
                GPIO.output(PUL, GPIO.HIGH)
                time.sleep(0.0001)
                GPIO.output(PUL, GPIO.LOW)
                time.sleep(0.0001)
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
