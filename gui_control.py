import tkinter as tk
import socket
import os
from dotenv import load_dotenv

# Load environment variables (useful if running locally with a .env file)
load_dotenv()

# Get IP and port from environment variables
motor_controller_ip = os.getenv('MOTOR_CONTROLLER_IP')
motor_controller_port = int(os.getenv('MOTOR_CONTROLLER_PORT'))

def send_command(command):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((motor_controller_ip, motor_controller_port))
        s.sendall(command.encode())
        s.close()

def run_motor():
    send_command("RUN")

# Create the GUI
root = tk.Tk()
root.title("Motor Control")

# Create a button to run the motor
run_button = tk.Button(root, text="Run Motor", command=run_motor, height=2, width=10)
run_button.pack(pady=20)

# Start the GUI loop
root.mainloop()

