import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt
import os
from dotenv import load_dotenv

# Load environment variables (for local testing with .env file)
load_dotenv()

# GPIO setup
PUL = 17
DIR = 27
steps = 1600

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.output(DIR, GPIO.HIGH)

# MQTT setup
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("motor/control")

def on_message(client, userdata, msg):
    if msg.topic == "motor/control":
        command = msg.payload.decode()
        if command == "RUN":
            run_motor()

def run_motor():
    for i in range(steps):
        GPIO.output(PUL, GPIO.HIGH)
        time.sleep(0.01)
        GPIO.output(PUL, GPIO.LOW)
        time.sleep(0.01)
    print("Motor run complete")
    GPIO.cleanup()

# Get IP from environment variables
mqtt_broker_ip = os.getenv('MOTOR_CONTROLLER_IP')
print(f"Connecting to MQTT Broker at IP: {mqtt_broker_ip}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(mqtt_broker_ip, 1883, 60)

client.loop_forever()
