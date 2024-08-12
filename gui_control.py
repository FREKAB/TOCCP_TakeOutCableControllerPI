import tkinter as tk
import math
import paho.mqtt.client as mqtt

# MQTT setup
mqtt_broker_ip = "192.168.10.9"  # Replace with your actual broker IP
client = mqtt.Client()
client.connect(mqtt_broker_ip, 1883, 60)

# List to hold the calculated rotations for each segment
rotation_list = []
current_segment = 0

def calculate_rotations():
    global rotation_list, current_segment
    rotation_list = []  # Reset the list
    current_segment = 0  # Reset the segment counter

    # Get user inputs
    D = float(inner_diameter_input.get())  # Inner diameter in mm
    d = float(cable_diameter_input.get())  # Cable diameter in mm
    L = float(cable_length_input.get()) * 1000  # Total cable length in mm
    stops = int(stop_interval_input.get())  # Number of stops (every x meters)
    
    # Calculate the number of meters per stop
    length_per_stop = L / stops
    
    # Initialize variables
    current_diameter = D

    # Calculate rotations for each segment
    for i in range(stops):
        # Calculate the current circumference
        current_circumference = math.pi * current_diameter
        
        # Calculate the number of rotations for this segment
        rotations = length_per_stop / current_circumference
        
        # Store the rotations required for this segment
        rotation_list.append(rotations)
        
        # Increase the diameter by twice the cable diameter for the next layer
        current_diameter += 2 * d

    # Display calculated rotations
    rotation_display.config(text=f"Calculated {len(rotation_list)} segments.")

def run_motor():
    global current_segment
    if current_segment < len(rotation_list):
        rotations = rotation_list[current_segment]
        client.publish("motor/control", str(rotations))  # Publish the number of rotations
        current_segment += 1
        rotation_display.config(text=f"Running segment {current_segment}/{len(rotation_list)}")
    else:
        rotation_display.config(text="All segments completed.")

# Create the GUI
root = tk.Tk()
root.title("Motor Control")

# Input fields for parameters
tk.Label(root, text="Inner Diameter (mm):").pack(pady=5)
inner_diameter_input = tk.Entry(root)
inner_diameter_input.pack(pady=5)

tk.Label(root, text="Cable Diameter (mm):").pack(pady=5)
cable_diameter_input = tk.Entry(root)
cable_diameter_input.pack(pady=5)

tk.Label(root, text="Cable Length (m):").pack(pady=5)
cable_length_input = tk.Entry(root)
cable_length_input.pack(pady=5)

tk.Label(root, text="Stop Interval (number of stops):").pack(pady=5)
stop_interval_input = tk.Entry(root)
stop_interval_input.pack(pady=5)

# Button to calculate rotations
calculate_button = tk.Button(root, text="Calculate Rotations", command=calculate_rotations, height=2, width=20)
calculate_button.pack(pady=10)

# Display the calculated rotations
rotation_display = tk.Label(root, text="")
rotation_display.pack(pady=10)

# Button to run the motor
run_button = tk.Button(root, text="Run/Continue", command=run_motor, height=2, width=20)
run_button.pack(pady=10)

# Start the GUI loop
root.mainloop()
