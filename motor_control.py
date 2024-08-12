import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import threading

# GPIO setup
PUL = 17
DIR = 27
steps = 1600

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)

def run_motor():
    GPIO.output(DIR, GPIO.HIGH)  # Set direction
    for i in range(steps):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.0001)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.0001)
    print("Motor run complete")
    # Don't call GPIO.cleanup() here if you want to be able to run the motor again

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    if msg.topic == "motor/control":
        command = msg.payload.decode()
        if command == "RUN":
            print("RUN command received")
            # Run the motor in a separate thread to avoid blocking
            threading.Thread(target=run_motor).start()

# MQTT setup
mqtt_broker_ip = "192.168.10.9"  # Replace with your broker IP
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(mqtt_broker_ip, 1883, 60)
client.loop_forever()  # Keep listening for incoming messages

