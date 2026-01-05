# Robot Control System - Separated Nodes

## Summary of Changes

Successfully separated the monolithic ESP32 communication node into two independent nodes:

### ✅ Robot Node
- **File:** `robot_node.py`
- **Class:** `RobotNode`
- **Purpose:** ESP32 serial communication, encoder reading, odometry calculation
- **No GUI** - pure ROS2 node running with `rclpy.spin()`
- **Status:** ✅ Built and installed

### ✅ Control Node
- **File:** `control_node.py`
- **Class:** `ControlNode`
- **Purpose:** tkinter GUI for robot control with direction buttons and speed slider
- **Publishes:** Commands to `/esp32_commands`
- **Subscribes:** Encoder feedback from `/esp32_encoders`
- **Status:** ✅ Built and installed

## Quick Start

### On Raspberry Pi (Robot):
```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch robot_package esp32_communication.launch.py
```

### On Laptop or RPi (Control):
```bash
cd ~/robot_ws
./launch_control_gui.sh
```

## Testing

You can verify the nodes work independently:

**Test Robot Node:**
```bash
# Terminal 1: Launch robot node
ros2 launch robot_package esp32_communication.launch.py

# Terminal 2: Send manual command
ros2 topic pub /esp32_commands std_msgs/msg/String "data: 'forward:150'" --once
```

**Test Control Node:**
```bash
# Terminal 1: Launch robot node (or have it running on RPi)
ros2 launch robot_package esp32_communication.launch.py

# Terminal 2: Launch control GUI
ros2 launch robot_package control_gui.launch.py
```

## Architecture

```
┌─────────────────┐                  ┌─────────────────┐
│  Control Node   │                  │   Robot Node    │
│  (control_node) │                  │  (robot_node)   │
├─────────────────┤                  ├─────────────────┤
│ - tkinter GUI   │                  │ - Serial Port   │
│ - Direction Btns│◄────Topics───────►│ - ESP32 Comm   │
│ - Speed Slider  │  /esp32_commands │ - Encoders      │
│ - Encoder Display│ /esp32_encoders │ - Odometry      │
└─────────────────┘                  │ - Joint States  │
                                     └─────────────────┘
                                            │
                                            ▼
                                     ┌─────────────────┐
                                     │     ESP32       │
                                     │ (Serial /dev/   │
                                     │  ttyUSB0)       │
                                     └─────────────────┘
```

## Files Modified/Created

### Created:
- ✅ `robot_node.py` - Clean robot node without GUI
- ✅ `control_node.py` - Standalone GUI control node
- ✅ `control_gui.launch.py` - Launch file for control GUI
- ✅ `launch_control_gui.sh` - Helper script
- ✅ `ARCHITECTURE.md` - Architecture documentation

### Modified:
- ✅ `CMakeLists.txt` - Added new nodes to install
- ✅ `esp32_communication.launch.py` - Updated to use robot_node.py

### Backed Up:
- ✅ `robot_node_backup.py` - Original file with GUI (for reference)

## Next Steps

1. **Test on RPi:** Copy the workspace to your Raspberry Pi and test the robot node
2. **Test GUI:** Run the control GUI on your laptop and verify network communication
3. **Configure Network:** Set `ROS_DOMAIN_ID=42` on both machines if using network control
4. **Commit Changes:** Add and commit the new files to git

```bash
cd ~/robot_ws
git add .
git commit -m "Separate robot_node and control_node architecture"
```
