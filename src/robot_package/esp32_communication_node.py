#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import serial
import time
import tkinter as tk
from tkinter import ttk
import threading
import math

class ESP32CommunicationNode(Node):
    def __init__(self):
        super().__init__('esp32_communication_node')

        # Publisher for encoder values
        self.encoder_publisher = self.create_publisher(String, 'esp32_encoders', 10)
        
        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        # Publisher for joint states (wheel rotation)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber for motor commands
        self.command_subscriber = self.create_subscription(
            String,
            'esp32_commands',
            self.command_callback,
            10
        )

        # Serial communication setup
        self.serial_port = None
        self.port_name = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.reconnect_interval = 5.0  # seconds
        self.last_reconnect_attempt = 0
        
        # Encoder values for GUI display
        self.encoder1_value = 0
        self.encoder2_value = 0
        self.speed_value = 50  # Default speed (0-100)
        
        # Odometry parameters (updated with actual robot dimensions)
        self.wheel_diameter = 0.06858  # meters (2.7 inches)
        self.wheel_base = 0.1778  # meters (7 inches - distance between wheels)
        self.encoder_resolution = 374  # pulses per wheel revolution (GA25-370: 11 PPR × 34 gear ratio)
        self.gear_ratio = 34.0  # GA25-370 gear ratio for 620 RPM
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.last_time = self.get_clock().now()
        
        # Velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Wheel angles for joint states (in radians)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        
        self.connect_serial()
        self.timer = self.create_timer(0.02, self.read_encoder_values)  # 20ms for real-time updates

    def connect_serial(self):
        """Attempt to connect to the serial port"""
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(
                self.port_name, 
                self.baudrate, 
                timeout=0.01,  # Very short timeout for real-time non-blocking reads
                write_timeout=0.1  # Fast write timeout
            )
            time.sleep(0.3)  # Give device time to initialize
            self.serial_port.reset_input_buffer()  # Clear any stale data
            self.serial_port.reset_output_buffer()
            self.get_logger().info(f'Successfully connected to {self.port_name} at {self.baudrate} baud')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port_name}: {e}')
            self.serial_port = None
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error while connecting to serial port: {e}')
            self.serial_port = None
            return False

    def read_encoder_values(self):
        """Read encoder values from ESP32"""
        # Try to reconnect if not connected
        if not self.serial_port or not self.serial_port.is_open:
            current_time = time.time()
            if current_time - self.last_reconnect_attempt >= self.reconnect_interval:
                self.get_logger().warn('Serial port not connected. Attempting to reconnect...')
                self.last_reconnect_attempt = current_time
                self.connect_serial()
            return

        try:
            # Read all available data for real-time updates
            while self.serial_port.in_waiting:
                data = self.serial_port.readline().decode('utf-8').strip()
                if data and data.startswith("ENCODER:"):
                    msg = String()
                    msg.data = data.replace("ENCODER:", "")
                    self.encoder_publisher.publish(msg)
                    
                    # Parse encoder values for GUI display
                    try:
                        parts = msg.data.split(',')
                        if len(parts) >= 2:
                            self.encoder1_value = int(parts[0].strip())
                            self.encoder2_value = int(parts[1].strip())
                            
                            # Calculate and publish odometry
                            self.update_odometry(self.encoder1_value, self.encoder2_value)
                    except (ValueError, IndexError):
                        pass
        except (serial.SerialException, OSError) as e:
            # Handle disconnection gracefully
            self.get_logger().warn(f'Serial port disconnected')
            try:
                if self.serial_port:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
        except UnicodeDecodeError:
            # Ignore decode errors, just skip this packet
            pass
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {e}')
            try:
                if self.serial_port:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
    
    def update_odometry(self, encoder_left, encoder_right):
        """Calculate odometry from encoder values"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt < 0.001:  # Avoid division by zero
            return
        
        # Calculate distance traveled by each wheel (negate both for correct direction)
        delta_right = (encoder_left - self.last_encoder_left)
        delta_left = -(encoder_right - self.last_encoder_right)
        
        # Convert encoder ticks to distance (meters)
        wheel_circumference = math.pi * self.wheel_diameter
        distance_per_tick = wheel_circumference / self.encoder_resolution
        
        distance_left = delta_left * distance_per_tick
        distance_right = delta_right * distance_per_tick
        
        # Calculate robot motion
        distance_center = (distance_left + distance_right) / 2.0
        delta_theta = (distance_right - distance_left) / self.wheel_base
        
        # Update position
        if abs(delta_theta) < 0.0001:  # Moving straight
            self.x += distance_center * math.cos(self.theta)
            self.y += distance_center * math.sin(self.theta)
        else:  # Moving in an arc
            radius = distance_center / delta_theta
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + delta_theta) - math.cos(self.theta))
        
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Calculate velocities
        self.linear_velocity = distance_center / dt
        self.angular_velocity = delta_theta / dt
        
        # Calculate wheel angles (radians) for joint states
        wheel_circumference = math.pi * self.wheel_diameter
        distance_per_tick = wheel_circumference / self.encoder_resolution
        
        self.left_wheel_angle = (encoder_left * distance_per_tick) / (self.wheel_diameter / 2.0)
        self.right_wheel_angle = (encoder_right * distance_per_tick) / (self.wheel_diameter / 2.0)
        
        # Publish odometry message
        self.publish_odometry(current_time)
        
        # Publish joint states
        self.publish_joint_states(current_time)
        
        # Update last values
        self.last_encoder_left = encoder_left
        self.last_encoder_right = encoder_right
        self.last_time = current_time
    
    def publish_odometry(self, current_time):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from theta)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Set velocities
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity
        
        self.odom_publisher.publish(odom_msg)
        
        # Publish TF transform
        self.publish_tf(current_time)
    
    def publish_tf(self, current_time):
        """Publish TF transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Set rotation (quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_joint_states(self, current_time):
        """Publish wheel joint states for visualization"""
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        # Negate left wheel to match the turning direction fix
        joint_state.position = [-self.left_wheel_angle, self.right_wheel_angle]
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_state_publisher.publish(joint_state)

    def command_callback(self, msg):
        """Send commands to ESP32"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Cannot send command: Serial port not connected')
            return

        try:
            command = msg.data
            self.serial_port.write(f"{command}\n".encode('utf-8'))
            self.get_logger().info(f'Sent command to ESP32: {command}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial port error while writing: {e}')
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'Unexpected error while writing to serial port: {e}')
    
    def send_command(self, command):
        """Send command directly (for GUI)"""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn('Cannot send command: Serial port not connected')
            return

        try:
            # Convert speed from 0-100 to 0-255 for ESP32
            speed_pwm = int((self.speed_value / 100.0) * 255)
            # Format: "command:speed" (lowercase command)
            full_command = f"{command.lower()}:{speed_pwm}"
            self.serial_port.write(f"{full_command}\n".encode('utf-8'))
            self.get_logger().info(f'Sent command to ESP32: {full_command}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial port error while writing: {e}')
            try:
                if self.serial_port:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
        except OSError as e:
            self.get_logger().warn(f'Serial port I/O error while writing: {e}')
            try:
                if self.serial_port:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'Unexpected error while writing to serial port: {e}')
            try:
                if self.serial_port:
                    self.serial_port.close()
            except:
                pass
            self.serial_port = None

    def __del__(self):
        """Cleanup on node destruction"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self.get_logger().info('Serial port closed')
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = ESP32CommunicationNode()
    
    # Create GUI
    root = tk.Tk()
    root.title("ESP32 Robot Control")
    root.geometry("500x600")
    
    # Main frame
    main_frame = ttk.Frame(root, padding="10")
    main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
    
    # Title
    title_label = ttk.Label(main_frame, text="ESP32 Robot Controller", font=('Arial', 16, 'bold'))
    title_label.grid(row=0, column=0, columnspan=3, pady=10)
    
    # Direction buttons
    control_frame = ttk.Frame(main_frame)
    control_frame.grid(row=1, column=0, columnspan=3, pady=20)
    
    def send_front():
        node.send_command("backward")
    
    def send_back():
        node.send_command("forward")
    
    def send_left():
        node.send_command("left")
    
    def send_right():
        node.send_command("right")
    
    def send_stop():
        node.send_command("stop")
    
    # Front button
    front_btn = tk.Button(control_frame, text="FRONT", width=10, height=2, 
                          bg='#4CAF50', fg='white', font=('Arial', 12, 'bold'),
                          command=send_front)
    front_btn.grid(row=0, column=1, padx=5, pady=5)
    
    # Left button
    left_btn = tk.Button(control_frame, text="LEFT", width=10, height=2,
                         bg='#2196F3', fg='white', font=('Arial', 12, 'bold'),
                         command=send_left)
    left_btn.grid(row=1, column=0, padx=5, pady=5)
    
    # Stop button
    stop_btn = tk.Button(control_frame, text="STOP", width=10, height=2,
                         bg='#f44336', fg='white', font=('Arial', 12, 'bold'),
                         command=send_stop)
    stop_btn.grid(row=1, column=1, padx=5, pady=5)
    
    # Right button
    right_btn = tk.Button(control_frame, text="RIGHT", width=10, height=2,
                          bg='#2196F3', fg='white', font=('Arial', 12, 'bold'),
                          command=send_right)
    right_btn.grid(row=1, column=2, padx=5, pady=5)
    
    # Back button
    back_btn = tk.Button(control_frame, text="BACK", width=10, height=2,
                         bg='#FF9800', fg='white', font=('Arial', 12, 'bold'),
                         command=send_back)
    back_btn.grid(row=2, column=1, padx=5, pady=5)
    
    # Speed control
    speed_frame = ttk.Frame(main_frame)
    speed_frame.grid(row=2, column=0, columnspan=3, pady=20)
    
    speed_label = ttk.Label(speed_frame, text="Speed Control:", font=('Arial', 12))
    speed_label.grid(row=0, column=0, columnspan=2, pady=5)
    
    speed_value_label = ttk.Label(speed_frame, text="50", font=('Arial', 14, 'bold'))
    speed_value_label.grid(row=1, column=0, columnspan=2, pady=5)
    
    def update_speed(val):
        speed = int(float(val))
        node.speed_value = speed
        speed_value_label.config(text=str(speed))
    
    speed_slider = ttk.Scale(speed_frame, from_=0, to=100, orient=tk.HORIZONTAL,
                            length=300, command=update_speed)
    speed_slider.set(50)
    speed_slider.grid(row=2, column=0, columnspan=2, pady=5)
    
    # Encoder display
    encoder_frame = ttk.LabelFrame(main_frame, text="Encoder Values", padding="10")
    encoder_frame.grid(row=3, column=0, columnspan=3, pady=20, sticky=(tk.W, tk.E))
    
    encoder1_label = ttk.Label(encoder_frame, text="Encoder 1:", font=('Arial', 11))
    encoder1_label.grid(row=0, column=0, sticky=tk.W, pady=5)
    
    encoder1_value_label = ttk.Label(encoder_frame, text="0", font=('Arial', 11, 'bold'))
    encoder1_value_label.grid(row=0, column=1, sticky=tk.W, pady=5, padx=10)
    
    encoder2_label = ttk.Label(encoder_frame, text="Encoder 2:", font=('Arial', 11))
    encoder2_label.grid(row=1, column=0, sticky=tk.W, pady=5)
    
    encoder2_value_label = ttk.Label(encoder_frame, text="0", font=('Arial', 11, 'bold'))
    encoder2_value_label.grid(row=1, column=1, sticky=tk.W, pady=5, padx=10)
    
    # Odometry display
    odom_frame = ttk.LabelFrame(main_frame, text="Odometry", padding="10")
    odom_frame.grid(row=4, column=0, columnspan=3, pady=10, sticky=(tk.W, tk.E))
    
    pos_x_label = ttk.Label(odom_frame, text="X Position:", font=('Arial', 10))
    pos_x_label.grid(row=0, column=0, sticky=tk.W, pady=3)
    pos_x_value = ttk.Label(odom_frame, text="0.00 m", font=('Arial', 10, 'bold'))
    pos_x_value.grid(row=0, column=1, sticky=tk.W, pady=3, padx=10)
    
    pos_y_label = ttk.Label(odom_frame, text="Y Position:", font=('Arial', 10))
    pos_y_label.grid(row=1, column=0, sticky=tk.W, pady=3)
    pos_y_value = ttk.Label(odom_frame, text="0.00 m", font=('Arial', 10, 'bold'))
    pos_y_value.grid(row=1, column=1, sticky=tk.W, pady=3, padx=10)
    
    theta_label = ttk.Label(odom_frame, text="Heading:", font=('Arial', 10))
    theta_label.grid(row=2, column=0, sticky=tk.W, pady=3)
    theta_value = ttk.Label(odom_frame, text="0.0°", font=('Arial', 10, 'bold'))
    theta_value.grid(row=2, column=1, sticky=tk.W, pady=3, padx=10)
    
    vel_linear_label = ttk.Label(odom_frame, text="Linear Vel:", font=('Arial', 10))
    vel_linear_label.grid(row=0, column=2, sticky=tk.W, pady=3, padx=(20, 0))
    vel_linear_value = ttk.Label(odom_frame, text="0.00 m/s", font=('Arial', 10, 'bold'))
    vel_linear_value.grid(row=0, column=3, sticky=tk.W, pady=3, padx=10)
    
    vel_angular_label = ttk.Label(odom_frame, text="Angular Vel:", font=('Arial', 10))
    vel_angular_label.grid(row=1, column=2, sticky=tk.W, pady=3, padx=(20, 0))
    vel_angular_value = ttk.Label(odom_frame, text="0.00 rad/s", font=('Arial', 10, 'bold'))
    vel_angular_value.grid(row=1, column=3, sticky=tk.W, pady=3, padx=10)
    
    # Status label
    status_label = ttk.Label(main_frame, text="Status: Ready", font=('Arial', 10))
    status_label.grid(row=5, column=0, columnspan=3, pady=10)
    
    def update_gui():
        """Update GUI with latest encoder values"""
        try:
            encoder1_value_label.config(text=str(node.encoder1_value))
            encoder2_value_label.config(text=str(node.encoder2_value))
            
            # Update odometry display
            pos_x_value.config(text=f"{node.x:.2f} m")
            pos_y_value.config(text=f"{node.y:.2f} m")
            theta_value.config(text=f"{math.degrees(node.theta):.1f}°")
            vel_linear_value.config(text=f"{node.linear_velocity:.2f} m/s")
            vel_angular_value.config(text=f"{node.angular_velocity:.2f} rad/s")
            
            # Update status
            if node.serial_port and node.serial_port.is_open:
                status_label.config(text="Status: Connected", foreground='green')
            else:
                status_label.config(text="Status: Disconnected", foreground='red')
            
            # Process ROS2 events - very fast for real-time
            rclpy.spin_once(node, timeout_sec=0.001)
            
            # Schedule next update (20ms for real-time updates)
            root.after(20, update_gui)
        except Exception as e:
            print(f"GUI update error: {e}")
            root.after(20, update_gui)
    
    # Start GUI update loop
    update_gui()
    
    # Run ROS2 in separate thread
    def ros_spin():
        rclpy.spin(node)
    
    # Handle window close
    def on_closing():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    # Start tkinter main loop
    root.mainloop()

if __name__ == '__main__':
    main()