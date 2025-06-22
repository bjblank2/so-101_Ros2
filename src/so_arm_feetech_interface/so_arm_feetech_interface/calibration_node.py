import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from std_msgs.msg import String
import yaml
import sys
import os
import math
import subprocess
import stat
import time
import numpy as np

# Add the workspace root to the Python path to find robot_devices
workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../../..'))
sys.path.append(workspace_root)

try:
    from robot_devices.motors.feetech import FeetechMotorsBus
    from robot_devices.motors.configs import FeetechMotorsBusConfig
except ImportError as e:
    print("Could not import Feetech motor modules.")
    print(f"PYTHONPATH: {sys.path}")
    print(f"Workspace root: {workspace_root}")
    print(f"Current file: {__file__}")
    print(f"Error: {e}")
    sys.exit(1)


def fix_serial_permissions(port):
    """Fix serial port permissions"""
    try:
        # Check if user is in dialout group
        result = subprocess.run(['groups'], capture_output=True, text=True)
        if 'dialout' not in result.stdout:
            subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', os.getenv('USER')], check=True)
            return False, "Added user to dialout group. Please log out and log back in, or run: newgrp dialout"
        
        # Set permissions on the port
        subprocess.run(['sudo', 'chmod', '666', port], check=True)
        subprocess.run(['sudo', 'chown', 'dialout:dialout', port], check=True)
        return True, "Permissions set successfully"
    except subprocess.CalledProcessError as e:
        return False, f"Failed to set permissions: {e}"


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        if not config_path:
            self.get_logger().error("'config_path' parameter not set. Please provide path to config.yaml")
            return

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            leader_params = config['leader_arm']['ros__parameters']
            follower_params = config['follower_arm']['ros__parameters']

        # Store joint names from config
        self.joint_names = list(leader_params['motors'].keys())
        self.calibration_offsets = {}
        self.home_positions = {}
        
        # Initialize motor connections for direct control
        self.get_logger().info("Initializing motor connections for calibration...")
        
        # Convert motor IDs from strings to integers for leader arm
        leader_motors_int = {}
        for motor_name, motor_config in leader_params['motors'].items():
            motor_id = int(motor_config[0])
            motor_model = motor_config[1]
            leader_motors_int[motor_name] = (motor_id, motor_model)
        
        # Convert motor IDs from strings to integers for follower arm
        follower_motors_int = {}
        for motor_name, motor_config in follower_params['motors'].items():
            motor_id = int(motor_config[0])
            motor_model = motor_config[1]
            follower_motors_int[motor_name] = (motor_id, motor_model)

        # Initialize leader arm connection
        self.get_logger().info("Connecting to leader arm...")
        success, message = fix_serial_permissions(leader_params['port'])
        if not success:
            self.get_logger().warning(f"Permission fix attempt: {message}")

        leader_bus_config = FeetechMotorsBusConfig(port=leader_params['port'], motors=leader_motors_int)
        self.leader_motors_bus = FeetechMotorsBus(leader_bus_config)
        
        try:
            self.leader_motors_bus.connect()
            self.get_logger().info("Successfully connected to leader arm")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to leader arm: {e}")
            raise

        # Initialize follower arm connection
        self.get_logger().info("Connecting to follower arm...")
        success, message = fix_serial_permissions(follower_params['port'])
        if not success:
            self.get_logger().warning(f"Permission fix attempt: {message}")

        follower_bus_config = FeetechMotorsBusConfig(port=follower_params['port'], motors=follower_motors_int)
        self.follower_motors_bus = FeetechMotorsBus(follower_bus_config)
        
        try:
            self.follower_motors_bus.connect()
            self.get_logger().info("Successfully connected to follower arm")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to follower arm: {e}")
            raise

        # Services for calibration
        self.calibrate_service = self.create_service(Trigger, 'calibrate_arms', self.calibrate_arms_callback)
        self.home_arms_service = self.create_service(Trigger, 'home_arms', self.home_arms_callback)
        self.set_home_service = self.create_service(Trigger, 'set_home_position', self.set_home_position_callback)
        self.match_positions_service = self.create_service(Trigger, 'match_positions', self.match_positions_callback)

        # Publishers and subscribers
        self.status_publisher = self.create_publisher(String, 'calibration_status', 10)
        self.leader_joint_subscriber = self.create_subscription(
            JointState, 'leader_joint_states', self.leader_joint_callback, 10
        )
        self.follower_joint_subscriber = self.create_subscription(
            JointState, 'follower_joint_states', self.follower_joint_callback, 10
        )

        # Store current joint states
        self.leader_joint_states = None
        self.follower_joint_states = None

        # Timer for status updates
        self.timer = self.create_timer(1.0, self.publish_status)

        # Perform automatic startup calibration
        self.perform_startup_calibration()

        self.get_logger().info("Calibration node started. Available services:")
        self.get_logger().info("  /calibrate_arms - Run full calibration routine")
        self.get_logger().info("  /home_arms - Move both arms to home position")
        self.get_logger().info("  /set_home_position - Set current position as home")
        self.get_logger().info("  /match_positions - Match follower to leader position")

    def perform_startup_calibration(self):
        """Automatically calibrate arms at startup by reading current positions and making them equivalent"""
        try:
            self.get_logger().info("Performing automatic startup calibration...")
            
            # Wait a moment for motors to be ready
            time.sleep(2.0)
            
            # Read current positions from both arms
            self.get_logger().info("Reading current positions from both arms...")
            leader_positions = self.leader_motors_bus.read("Present_Position")
            follower_positions = self.follower_motors_bus.read("Present_Position")
            
            self.get_logger().info(f"Leader arm positions: {leader_positions}")
            self.get_logger().info(f"Follower arm positions: {follower_positions}")
            
            # Calculate the difference and set follower to match leader
            self.get_logger().info("Setting follower arm to match leader arm position...")
            for i, name in enumerate(self.joint_names):
                if i < len(leader_positions) and i < len(follower_positions):
                    leader_pos = leader_positions[i]
                    follower_pos = follower_positions[i]
                    
                    # Set follower to leader position
                    self.get_logger().info(f"Setting {name}: Leader={leader_pos}, Follower={follower_pos} -> {leader_pos}")
                    self.follower_motors_bus.write("Goal_Position", [leader_pos], motor_names=[name])
            
            # Wait for movement to complete
            time.sleep(3.0)
            
            # Verify the positions are now matched
            self.get_logger().info("Verifying calibration...")
            new_leader_positions = self.leader_motors_bus.read("Present_Position")
            new_follower_positions = self.follower_motors_bus.read("Present_Position")
            
            max_diff = 0
            for i, name in enumerate(self.joint_names):
                if i < len(new_leader_positions) and i < len(new_follower_positions):
                    diff = abs(new_leader_positions[i] - new_follower_positions[i])
                    max_diff = max(max_diff, diff)
                    self.get_logger().info(f"{name}: Leader={new_leader_positions[i]}, Follower={new_follower_positions[i]}, Diff={diff}")
            
            self.get_logger().info(f"Startup calibration completed! Maximum position difference: {max_diff}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to perform startup calibration: {e}")

    def leader_joint_callback(self, msg):
        self.leader_joint_states = msg

    def follower_joint_callback(self, msg):
        self.follower_joint_states = msg

    def publish_status(self):
        status_msg = String()
        if self.leader_joint_states and self.follower_joint_states:
            # Calculate position differences
            differences = []
            for i, name in enumerate(self.leader_joint_states.name):
                if name in self.follower_joint_states.name:
                    leader_pos = self.leader_joint_states.position[i]
                    follower_idx = self.follower_joint_states.name.index(name)
                    follower_pos = self.follower_joint_states.position[follower_idx]
                    diff = abs(leader_pos - follower_pos)
                    differences.append(diff)
            
            if differences:
                max_diff = max(differences)
                avg_diff = sum(differences) / len(differences)
                status_msg.data = f"Position differences - Max: {max_diff:.3f} rad, Avg: {avg_diff:.3f} rad"
            else:
                status_msg.data = "No matching joints found"
        else:
            status_msg.data = "Waiting for joint states..."
        
        self.status_publisher.publish(status_msg)

    def calibrate_arms_callback(self, request, response):
        """Full calibration routine"""
        try:
            self.get_logger().info("Starting full calibration routine...")
            
            # Read current positions from both arms
            self.get_logger().info("Reading current positions...")
            leader_positions = self.leader_motors_bus.read("Present_Position")
            follower_positions = self.follower_motors_bus.read("Present_Position")
            
            # Calculate calibration offsets
            self.get_logger().info("Calculating calibration offsets...")
            for i, name in enumerate(self.joint_names):
                if i < len(leader_positions) and i < len(follower_positions):
                    leader_pos = leader_positions[i]
                    follower_pos = follower_positions[i]
                    offset = leader_pos - follower_pos
                    self.calibration_offsets[name] = offset
                    self.get_logger().info(f"  {name}: Leader={leader_pos}, Follower={follower_pos}, Offset={offset}")
            
            response.success = True
            response.message = f"Calibration completed. Offsets calculated for {len(self.calibration_offsets)} joints."
            self.get_logger().info("Calibration completed successfully!")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to calibrate arms: {str(e)}"
            self.get_logger().error(f"Failed to calibrate arms: {e}")
        
        return response

    def home_arms_callback(self, request, response):
        """Move both arms to home position"""
        try:
            self.get_logger().info("Moving arms to home position...")
            
            # Default home positions (all joints at 0)
            home_positions = [0] * len(self.joint_names)
            
            # Move both arms to home
            self.leader_motors_bus.write("Goal_Position", home_positions, motor_names=self.joint_names)
            self.follower_motors_bus.write("Goal_Position", home_positions, motor_names=self.joint_names)
            
            response.success = True
            response.message = "Arms moved to home position"
            self.get_logger().info("Home command sent!")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to home arms: {str(e)}"
            self.get_logger().error(f"Failed to home arms: {e}")
        
        return response

    def set_home_position_callback(self, request, response):
        """Set current position as home position"""
        try:
            self.get_logger().info("Setting current position as home position...")
            
            # Read current positions from both arms
            leader_positions = self.leader_motors_bus.read("Present_Position")
            follower_positions = self.follower_motors_bus.read("Present_Position")
            
            # Store as home positions
            for i, name in enumerate(self.joint_names):
                if i < len(leader_positions) and i < len(follower_positions):
                    self.home_positions[name] = {
                        'leader': leader_positions[i],
                        'follower': follower_positions[i]
                    }
            
            response.success = True
            response.message = f"Home position set for {len(self.home_positions)} joints"
            self.get_logger().info("Home position set successfully!")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to set home position: {str(e)}"
            self.get_logger().error(f"Failed to set home position: {e}")
        
        return response

    def match_positions_callback(self, request, response):
        """Match follower arm position to leader arm position"""
        try:
            self.get_logger().info("Matching follower position to leader position...")
            
            # Read current leader positions
            leader_positions = self.leader_motors_bus.read("Present_Position")
            
            # Set follower to match leader
            for i, name in enumerate(self.joint_names):
                if i < len(leader_positions):
                    leader_pos = leader_positions[i]
                    self.follower_motors_bus.write("Goal_Position", [leader_pos], motor_names=[name])
                    self.get_logger().info(f"Matching {name}: {leader_pos}")
            
            response.success = True
            response.message = "Follower arm matched to leader position"
            self.get_logger().info("Match command sent!")
                
        except Exception as e:
            response.success = False
            response.message = f"Failed to match positions: {str(e)}"
            self.get_logger().error(f"Failed to match positions: {e}")
        
        return response

    def destroy_node(self):
        self.get_logger().info("Shutting down calibration node, disconnecting from motors.")
        if hasattr(self, 'leader_motors_bus'):
            self.leader_motors_bus.disconnect()
        if hasattr(self, 'follower_motors_bus'):
            self.follower_motors_bus.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 