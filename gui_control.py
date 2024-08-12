import tkinter as tk
import paho.mqtt.client as mqtt
import os
from dotenv import load_dotenv

# Load environment variables (for local testing with .env file)
load_dotenv()

# Get IP from environment variables
mqtt_broker_ip = os.getenv('MOTOR_CONTROLLER_IP')

# Setup MQTT client
client = mqtt.Client()
client.connect(mqtt_broker_ip, 1883, 60)

def run_motor():
    client.publish("motor/control", "RUN")

# Create the GUI
root = tk.Tk()
root.title("Motor Control")

# Create a button to run the motor
run_button = tk.Button(root, text="Run Motor", command=run_motor, height=2, width=10)
run_button.pack(pady=20)

# Start the GUI loop
root.mainloop()
