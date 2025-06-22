# Leader-Follower Arm Teleoperation with Calibration

This package provides leader-follower teleoperation functionality for the SO-101 robot arm using FeeTech STS3215 servos, including a comprehensive calibration system to ensure accurate position matching between leader and follower arms.

## Overview

The leader-follower system consists of three main nodes:

1. **Leader Arm Node** (`leader_arm_node`): Reads joint states from a leader arm and publishes them
2. **Follower Arm Node** (`follower_arm_node`): Subscribes to leader joint states and commands the follower arm to follow
3. **Calibration Node** (`calibration_node`): Provides calibration services to align both arms

## Hardware Setup

You'll need two SO-101 arms with FeeTech servos:
- **Leader Arm**: Connected to `/dev/ttyACM0` (or configure as needed)
- **Follower Arm**: Connected to `/dev/ttyACM1` (or configure as needed)

## Configuration

The system uses `leader_follower_config.yaml` which contains configurations for both arms and calibration settings:

```yaml
leader_arm:
  ros__parameters:
    port: "/dev/ttyACM0"  # Leader arm serial port
    motors:
      Rotation: [1, "sts3215"]
      Pitch: [2, "sts3215"]
      Elbow: [3, "sts3215"]
      Wrist_Pitch: [4, "sts3215"]
      Wrist_Roll: [5, "sts3215"]
      Jaw: [6, "sts3215"]
    publish_rate: 10.0

follower_arm:
  ros__parameters:
    port: "/dev/ttyACM1"  # Follower arm serial port
    motors:
      Rotation: [1, "sts3215"]
      Pitch: [2, "sts3215"]
      Elbow: [3, "sts3215"]
      Wrist_Pitch: [4, "sts3215"]
      Wrist_Roll: [5, "sts3215"]
      Jaw: [6, "sts3215"]
    publish_rate: 10.0
    scaling_factor: 1.0  # Scale leader movements
    enable_teleop: true  # Enable/disable teleoperation
    use_calibration: true  # Enable/disable calibration offsets

calibration:
  ros__parameters:
    home_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Home position in degrees
    test_positions: [0.0, 30.0, 60.0, 0.0, 0.0, 0.0]  # Test positions for verification
    position_tolerance: 2.0  # Tolerance for position matching (degrees)
    max_offset: 30.0  # Maximum allowed offset (degrees)
```

## Usage

### Running Complete System with Calibration

```bash
# Source the workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch leader, follower, and calibration nodes
ros2 launch so_arm_feetech_interface leader_follower_with_calibration.launch.py
```

### Running Arms Individually

```bash
# Run only the leader arm
ros2 launch so_arm_feetech_interface leader_arm.launch.py

# Run only the follower arm
ros2 launch so_arm_feetech_interface follower_arm.launch.py

# Run only the calibration node
ros2 run so_arm_feetech_interface calibration_node --ros-args -p config_path:=/path/to/leader_follower_config.yaml
```

### Running Nodes Directly

```bash
# Run leader arm node
ros2 run so_arm_feetech_interface leader_arm_node --ros-args -p config_path:=/path/to/leader_follower_config.yaml

# Run follower arm node
ros2 run so_arm_feetech_interface follower_arm_node --ros-args -p config_path:=/path/to/leader_follower_config.yaml

# Run calibration node
ros2 run so_arm_feetech_interface calibration_node --ros-args -p config_path:=/path/to/leader_follower_config.yaml
```

## Calibration System

The calibration system provides several services to ensure accurate position matching:

### Available Services

1. **`/calibrate_arms`** - Full calibration routine
   - Moves both arms to home position
   - Reads current positions and calculates offsets
   - Tests calibration with verification movements
   - Returns calculated calibration offsets

2. **`/home_arms`** - Move both arms to home position
   - Moves both arms to predefined home positions
   - Useful for initial setup and safety

3. **`/set_home_position`** - Set current position as home
   - Records current positions of both arms as home
   - Useful for custom home positions

4. **`/match_positions`** - Match follower to leader position
   - Immediately matches follower arm to current leader position
   - Applies calibration offsets if available

### Using Calibration Services

#### Method 1: Using the Calibration Client

```bash
# Run the calibration client
ros2 run so_arm_feetech_interface calibration_client
```

The client provides methods to call all calibration services:
- `calibrate()` - Run full calibration routine
- `home_arms()` - Move both arms to home position
- `set_home()` - Set current position as home
- `match_positions()` - Match follower to leader position

#### Method 2: Using ROS 2 Service Calls

```bash
# Run full calibration
ros2 service call /calibrate_arms std_srvs/srv/Trigger

# Move arms to home position
ros2 service call /home_arms std_srvs/srv/Trigger

# Set current position as home
ros2 service call /set_home_position std_srvs/srv/Trigger

# Match follower to leader position
ros2 service call /match_positions std_srvs/srv/Trigger
```

### Calibration Workflow

1. **Initial Setup**:
   ```bash
   # Launch the complete system
   ros2 launch so_arm_feetech_interface leader_follower_with_calibration.launch.py
   ```

2. **Move to Safe Position**:
   ```bash
   # Move both arms to home position
   ros2 service call /home_arms std_srvs/srv/Trigger
   ```

3. **Run Calibration**:
   ```bash
   # Run full calibration routine
   ros2 service call /calibrate_arms std_srvs/srv/Trigger
   ```

4. **Test Calibration**:
   ```bash
   # Test position matching
   ros2 service call /match_positions std_srvs/srv/Trigger
   ```

5. **Start Teleoperation**:
   - The follower arm will now follow the leader arm with calibration offsets applied

## Topics

### Leader Arm Node
- **Publishes**: `leader_joint_states` (sensor_msgs/JointState)
  - Current joint positions and velocities of the leader arm

### Follower Arm Node
- **Subscribes**: 
  - `leader_joint_states` (sensor_msgs/JointState) - Receives joint states from the leader arm
  - `calibration_status` (std_msgs/String) - Receives calibration status updates
- **Publishes**: `follower_joint_states` (sensor_msgs/JointState)
  - Current joint positions and velocities of the follower arm

### Calibration Node
- **Subscribes**: 
  - `leader_joint_states` (sensor_msgs/JointState)
  - `follower_joint_states` (sensor_msgs/JointState)
- **Publishes**: `calibration_status` (std_msgs/String)
  - Status updates and position differences between arms
- **Services**:
  - `/calibrate_arms` (std_srvs/srv/Trigger)
  - `/home_arms` (std_srvs/srv/Trigger)
  - `/set_home_position` (std_srvs/srv/Trigger)
  - `/match_positions` (std_srvs/srv/Trigger)

## Testing

### Monitor System Status

```bash
# Monitor both arms
ros2 run so_arm_feetech_interface test_leader_follower

# Monitor calibration status
ros2 topic echo /calibration_status
```

### Test Calibration

```bash
# Run calibration client for interactive testing
ros2 run so_arm_feetech_interface calibration_client
```

## Configuration Options

### Scaling Factor
The `scaling_factor` parameter in the follower arm configuration allows you to scale the leader's movements:
- `1.0`: Same size movements (default)
- `0.5`: Half-size movements
- `2.0`: Double-size movements

### Calibration Settings
- `use_calibration`: Enable/disable calibration offsets in follower arm
- `position_tolerance`: Tolerance for position matching (degrees)
- `max_offset`: Maximum allowed calibration offset (degrees)

### Enable/Disable Features
- `enable_teleop`: Enable/disable teleoperation in follower arm
- `use_calibration`: Enable/disable calibration offsets in follower arm

## Safety Features

1. **Position Tolerance**: Calibration includes position tolerance checking
2. **Maximum Offset**: Limits maximum allowed calibration offset
3. **Home Position**: Safe home position for both arms
4. **Emergency Stop**: Always have an emergency stop mechanism available

## Troubleshooting

### Serial Port Permissions
If you encounter permission errors:
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```
Then log out and log back in, or run `newgrp dialout`.

### Calibration Issues
1. **Large Position Differences**: Check if arms are in similar starting positions
2. **Calibration Fails**: Ensure both arms are connected and responding
3. **Offset Too Large**: Check for mechanical issues or incorrect joint limits

### Joint Name Mismatch
Ensure that the joint names in the configuration match the URDF joint names. The current configuration uses:
- `Rotation`
- `Pitch`
- `Elbow`
- `Wrist_Pitch`
- `Wrist_Roll`
- `Jaw`

### Motor IDs
Make sure the motor IDs in the configuration match the actual servo IDs on your hardware. The default configuration assumes IDs 1-6.

## Safety Notes

1. **Always calibrate before teleoperation** to ensure safe and accurate following
2. **Monitor position differences** using the calibration status topic
3. **Use appropriate scaling factors** to prevent excessive movements
4. **Have emergency stop mechanisms** available during operation
5. **Test in a safe environment** before using in production
6. **Regular calibration** may be needed due to wear and temperature changes 