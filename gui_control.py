import tkinter as tk
import paho.mqtt.client as mqtt

# MQTT setup
mqtt_broker_ip = "192.168.10.9"  # Replace with your actual broker IP
client = mqtt.Client()
client.connect(mqtt_broker_ip, 1883, 60)

def run_motor():
    rotations = rotation_input.get()  # Get the number of rotations from the input field
    if rotations.isdigit():
        client.publish("motor/control", rotations)  # Publish the number of rotations
    else:
        print("Please enter a valid number of rotations")

# Create the GUI
root = tk.Tk()
root.title("Motor Control")

# Add a label and input field for rotations
tk.Label(root, text="Enter rotations:").pack(pady=10)
rotation_input = tk.Entry(root)
rotation_input.pack(pady=10)

# Create a button to run the motor
run_button = tk.Button(root, text="Run Motor", command=run_motor, height=2, width=10)
run_button.pack(pady=20)

# Start the GUI loop
root.mainloop()
