import tkinter as tk
import paho.mqtt.client as mqtt

# Setup MQTT client
client = mqtt.Client()
client.connect("broker_ip_address", 1883, 60)  # Replace with broker IP

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
