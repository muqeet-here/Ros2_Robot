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
import math

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

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
                    
                    # Parse encoder values for odometry
                    try:
                        parts = msg.data.split(',')
                        if len(parts) >= 2:
                            encoder1 = int(parts[0].strip())
                            encoder2 = int(parts[1].strip())
                            
                            # Calculate and publish odometry
                            self.update_odometry(encoder1, encoder2)
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
    node = RobotNode()
    
    print("\n" + "="*50)
    print("Robot Node Started")
    print("="*50)
    print(f"\nListening for commands on /esp32_commands topic")
    print(f"Publishing encoder data to /esp32_encoders topic")
    print(f"Publishing odometry to /odom topic")
    print(f"Serial port: {node.port_name}")
    print("="*50 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
