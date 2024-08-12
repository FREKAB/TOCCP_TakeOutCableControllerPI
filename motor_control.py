import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt

# GPIO setup
PUL = 17
DIR = 27
steps = 1600

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL, GPIO.OUT)
GPIO.setup(DIR, GPIO.OUT)
GPIO.output(DIR, GPIO.HIGH)

# Define MQTT events
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

# Setup MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("broker_ip_address", 1883, 60)  # Replace with broker IP

client.loop_forever()
