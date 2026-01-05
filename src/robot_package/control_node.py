#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Publisher for robot commands
        self.command_publisher = self.create_publisher(String, 'esp32_commands', 10)
        
        # Subscriber for encoder feedback
        self.encoder_subscriber = self.create_subscription(
            String,
            'esp32_encoders',
            self.encoder_callback,
            10
        )
        
        # Control variables
        self.speed_value = 50  # Default speed (0-100)
        self.encoder1_value = 0
        self.encoder2_value = 0
        
        self.get_logger().info('Control Node started')
    
    def encoder_callback(self, msg):
        """Update encoder values from robot"""
        try:
            parts = msg.data.split(',')
            if len(parts) >= 2:
                self.encoder1_value = int(parts[0].strip())
                self.encoder2_value = int(parts[1].strip())
        except (ValueError, IndexError):
            pass
    
    def send_command(self, command):
        """Send command to robot"""
        msg = String()
        # Convert speed from 0-100 to 0-255 for ESP32
        speed_pwm = int((self.speed_value / 100.0) * 255)
        msg.data = f"{command.lower()}:{speed_pwm}"
        self.command_publisher.publish(msg)
        self.get_logger().info(f'Sent command: {msg.data}')

def create_gui(node):
    """Create the control GUI"""
    root = tk.Tk()
    root.title("Robot Control Node")
    root.geometry("400x500")
    
    # Main frame
    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Title
    title_label = ttk.Label(main_frame, text="Robot Control", font=('Arial', 16, 'bold'))
    title_label.pack(pady=10)
    
    # Direction buttons
    control_frame = ttk.Frame(main_frame)
    control_frame.pack(pady=20)
    
    def send_stop(event=None):
        node.send_command("stop")
    
    # Front button
    front_btn = tk.Button(control_frame, text="FRONT", width=10, height=2,
                          bg='#4CAF50', fg='white', font=('Arial', 12, 'bold'))
    front_btn.grid(row=0, column=1, padx=5, pady=5)
    front_btn.bind('<ButtonPress-1>', lambda e: node.send_command("forward"))
    front_btn.bind('<ButtonRelease-1>', send_stop)
    
    # Left button
    left_btn = tk.Button(control_frame, text="LEFT", width=10, height=2,
                         bg='#2196F3', fg='white', font=('Arial', 12, 'bold'))
    left_btn.grid(row=1, column=0, padx=5, pady=5)
    left_btn.bind('<ButtonPress-1>', lambda e: node.send_command("left"))
    left_btn.bind('<ButtonRelease-1>', send_stop)
    
    # Stop button
    stop_btn = tk.Button(control_frame, text="STOP", width=10, height=2,
                         bg='#f44336', fg='white', font=('Arial', 12, 'bold'),
                         command=send_stop)
    stop_btn.grid(row=1, column=1, padx=5, pady=5)
    
    # Right button
    right_btn = tk.Button(control_frame, text="RIGHT", width=10, height=2,
                          bg='#2196F3', fg='white', font=('Arial', 12, 'bold'))
    right_btn.grid(row=1, column=2, padx=5, pady=5)
    right_btn.bind('<ButtonPress-1>', lambda e: node.send_command("right"))
    right_btn.bind('<ButtonRelease-1>', send_stop)
    
    # Back button
    back_btn = tk.Button(control_frame, text="BACK", width=10, height=2,
                         bg='#FF9800', fg='white', font=('Arial', 12, 'bold'))
    back_btn.bind('<ButtonPress-1>', lambda e: node.send_command("backward"))
    back_btn.bind('<ButtonRelease-1>', send_stop)
    back_btn.grid(row=2, column=1, padx=5, pady=5)
    
    # Speed control
    speed_frame = ttk.LabelFrame(main_frame, text="Speed Control", padding="10")
    speed_frame.pack(fill=tk.X, pady=20)
    
    speed_value_label = ttk.Label(speed_frame, text="50", font=('Arial', 14, 'bold'))
    speed_value_label.pack(pady=5)
    
    def update_speed(val):
        speed = int(float(val))
        node.speed_value = speed
        speed_value_label.config(text=str(speed))
    
    speed_slider = ttk.Scale(speed_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                            length=300, command=update_speed)
    speed_slider.set(50)
    speed_slider.pack(pady=5)
    
    # Encoder display
    encoder_frame = ttk.LabelFrame(main_frame, text="Encoder Values", padding="10")
    encoder_frame.pack(fill=tk.X, pady=10)
    
    encoder1_label = ttk.Label(encoder_frame, text="Encoder 1:", font=('Arial', 11))
    encoder1_label.grid(row=0, column=0, sticky=tk.W, pady=5)
    
    encoder1_value_label = ttk.Label(encoder_frame, text="0", font=('Arial', 11, 'bold'))
    encoder1_value_label.grid(row=0, column=1, sticky=tk.W, pady=5, padx=10)
    
    encoder2_label = ttk.Label(encoder_frame, text="Encoder 2:", font=('Arial', 11))
    encoder2_label.grid(row=1, column=0, sticky=tk.W, pady=5)
    
    encoder2_value_label = ttk.Label(encoder_frame, text="0", font=('Arial', 11, 'bold'))
    encoder2_value_label.grid(row=1, column=1, sticky=tk.W, pady=5, padx=10)
    
    def update_gui():
        """Update GUI with latest values"""
        try:
            encoder1_value_label.config(text=str(node.encoder1_value))
            encoder2_value_label.config(text=str(node.encoder2_value))
            
            # Process ROS2 events
            rclpy.spin_once(node, timeout_sec=0.001)
            
            # Schedule next update
            root.after(50, update_gui)
        except Exception as e:
            print(f"GUI update error: {e}")
            root.after(50, update_gui)
    
    def on_closing():
        """Handle window close"""
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Start GUI update loop
    update_gui()
    
    return root

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    
    print("\n" + "="*50)
    print("Robot Control Node Started")
    print("="*50)
    print("\nOpening control GUI...")
    print("Use the buttons to control the robot")
    print("="*50 + "\n")
    
    # Create and run GUI
    root = create_gui(node)
    root.mainloop()

if __name__ == '__main__':
    main()
