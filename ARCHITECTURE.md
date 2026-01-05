# Robot Package Architecture

## Overview

The robot control system has been separated into two independent nodes for better modularity:

### 1. Robot Node (`robot_node.py`)
**Purpose:** Handles all ESP32 communication and robot hardware interfaces  
**Runs on:** Raspberry Pi (on the robot)  
**Topics:**
- **Subscribes to:** `/esp32_commands` (String) - receives motor commands
- **Publishes:**
  - `/esp32_encoders` (String) - raw encoder data
  - `/odom` (Odometry) - calculated odometry
  - `/joint_states` (JointState) - wheel joint states
  - `/tf` - transform from odom to base_link

### 2. Control Node (`control_node.py`)
**Purpose:** Provides GUI interface for robot control  
**Runs on:** Laptop or Raspberry Pi  
**Topics:**
- **Publishes to:** `/esp32_commands` (String) - sends motor commands (format: `command:speed`)
- **Subscribes to:** `/esp32_encoders` (String) - receives encoder feedback for display

## Launch Files

### For Raspberry Pi (Robot)
```bash
# Launch the robot node (ESP32 communication + odometry)
ros2 launch robot_package esp32_communication.launch.py
```

### For Control Station (Laptop or RPi)
```bash
# Launch the control GUI
ros2 launch robot_package control_gui.launch.py

# Or use the helper script:
./launch_control_gui.sh
```

## Network Configuration

If running Control Node on a separate machine (laptop), ensure both machines are on the same network and using the same `ROS_DOMAIN_ID`:

```bash
export ROS_DOMAIN_ID=42
```

Add this to your `~/.bashrc` or uncomment it in the launch scripts.

## Command Format

Commands sent to `/esp32_commands` should follow this format:
```
command:speed
```

Where:
- `command` can be: `forward`, `backward`, `left`, `right`, `stop`
- `speed` is PWM value (0-255)

Examples:
- `forward:150` - move forward at speed 150
- `left:200` - turn left at speed 200
- `stop:0` - stop motors

## Architecture Benefits

1. **Separation of Concerns:** Robot hardware logic separate from control interface
2. **Flexibility:** Control GUI can run anywhere on the network
3. **Multiple Controllers:** Multiple control nodes can send commands (e.g., GUI + autonomous)
4. **Easier Testing:** Can test robot node without GUI or vice versa
5. **Maintainability:** Simpler codebase for each component

## Legacy Support

The original monolithic `esp32_communication_node.py` is still available if needed:
```bash
ros2 run robot_package esp32_communication_node.py
```

## Files

- `robot_node.py` - Robot Node (ESP32 communication)
- `control_node.py` - Control Node (GUI)
- `esp32_communication.launch.py` - Launch file for robot node
- `control_gui.launch.py` - Launch file for control GUI
- `launch_control_gui.sh` - Helper script for control GUI
