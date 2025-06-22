import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml
import sys
import os
import math
import subprocess
import stat

# Add the workspace root to the Python path to find robot_devices
# The node is installed in install/so_arm_feetech_interface/lib/python3.10/site-packages/so_arm_feetech_interface/
# We need to go up 5 levels to reach the workspace root
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
    """Fix serial port permissions automatically"""
    try:
        # Check if port exists
        if not os.path.exists(port):
            return False, f"Port {port} does not exist"
        
        # Get current user
        current_user = os.getenv('USER')
        if not current_user:
            return False, "Could not determine current user"
        
        # Add user to dialout group (this requires sudo)
        try:
            subprocess.run(['sudo', 'usermod', '-a', '-G', 'dialout', current_user], 
                         check=True, capture_output=True)
            print(f"Added user {current_user} to dialout group")
        except subprocess.CalledProcessError:
            print(f"Warning: Could not add user to dialout group. You may need to run: sudo usermod -a -G dialout {current_user}")
        
        # Set permissions on the port
        try:
            subprocess.run(['sudo', 'chmod', '666', port], check=True, capture_output=True)
            print(f"Set permissions on {port}")
        except subprocess.CalledProcessError:
            print(f"Warning: Could not set permissions on {port}")
        
        # Set ownership to dialout group
        try:
            subprocess.run(['sudo', 'chown', 'root:dialout', port], check=True, capture_output=True)
            print(f"Set ownership of {port} to dialout group")
        except subprocess.CalledProcessError:
            print(f"Warning: Could not set ownership of {port}")
        
        return True, "Permissions fixed successfully"
        
    except Exception as e:
        return False, f"Error fixing permissions: {e}"


class FollowerArmNode(Node):
    def __init__(self):
        super().__init__('follower_arm_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        if not config_path:
            self.get_logger().error("'config_path' parameter not set. Please provide path to config.yaml")
            return

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            params = config['follower_arm']['ros__parameters']

        port = params['port']
        motors = params['motors']
        
        # Convert motor IDs from strings to integers
        motors_int = {}
        for motor_name, motor_config in motors.items():
            # motor_config is a list: ["motor_id", "model"]
            motor_id = int(motor_config[0])  # First element is motor_id
            motor_model = motor_config[1]    # Second element is model
            motors_int[motor_name] = (motor_id, motor_model)
        
        self.publish_rate = params.get('publish_rate', 10.0)
        self.calibration_data = params.get('calibration')
        self.scaling_factor = params.get('scaling_factor', 1.0)  # Scale leader movements
        self.enable_teleop = params.get('enable_teleop', True)
        self.use_calibration = params.get('use_calibration', True)  # Use calibration offsets

        self.get_logger().info(f"Connecting to follower arm on port '{port}'...")

        # Try to fix permissions first
        success, message = fix_serial_permissions(port)
        if not success:
            self.get_logger().warning(f"Permission fix attempt: {message}")
            self.get_logger().warning("You may need to run: sudo usermod -a -G dialout $USER && sudo chmod 666 " + port)

        bus_config = FeetechMotorsBusConfig(port=port, motors=motors_int)
        self.motors_bus = FeetechMotorsBus(bus_config)
        
        try:
            self.motors_bus.connect()
        except Exception as e:
            if "Permission denied" in str(e):
                self.get_logger().error(f"Permission denied on {port}. Please run the following commands:")
                self.get_logger().error(f"sudo usermod -a -G dialout $USER")
                self.get_logger().error(f"sudo chmod 666 {port}")
                self.get_logger().error("Then log out and log back in, or run: newgrp dialout")
                raise
            else:
                raise

        if self.calibration_data:
            self.get_logger().info("Applying calibration data.")
            self.motors_bus.set_calibration(self.calibration_data)

        self.joint_names = list(self.motors_bus.motor_names)
        self.get_logger().info(f"Successfully connected to follower arm motors: {self.joint_names}")

        # Subscribe to leader joint states
        self.leader_joint_state_subscriber = self.create_subscription(
            JointState, 'leader_joint_states', self._leader_joint_state_callback, 10
        )

        # Subscribe to calibration status
        self.calibration_status_subscriber = self.create_subscription(
            String, 'calibration_status', self._calibration_status_callback, 10
        )

        # Publish follower arm joint states
        self.follower_joint_state_publisher = self.create_publisher(JointState, 'follower_joint_states', 10)

        # Timer for publishing follower joint states
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_follower_joint_states_callback)

        # Store the latest leader joint state and calibration data
        self.latest_leader_joint_state = None
        self.calibration_offsets = {}
        self.calibration_status = "No calibration data"

        self.get_logger().info(f"Follower arm node started. Teleop enabled: {self.enable_teleop}, Calibration enabled: {self.use_calibration}")

    def _calibration_status_callback(self, msg):
        """Callback for calibration status updates"""
        self.calibration_status = msg.data
        # Parse calibration offsets from status message if available
        # This is a simple implementation - in practice you might want a more robust method
        if "Offsets:" in msg.data:
            try:
                # Extract offsets from message (this is a simplified approach)
                offset_str = msg.data.split("Offsets:")[1].strip()
                # Parse the offset string to extract individual joint offsets
                # This would need to be implemented based on the actual format
                self.get_logger().debug(f"Received calibration offsets: {offset_str}")
            except Exception as e:
                self.get_logger().debug(f"Could not parse calibration offsets: {e}")

    def _leader_joint_state_callback(self, msg: JointState):
        """Callback for leader joint states - commands the follower arm"""
        if not self.enable_teleop:
            return

        self.latest_leader_joint_state = msg
        
        motor_names_to_write = []
        goal_positions = []

        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                motor_names_to_write.append(name)
                # Apply scaling factor and convert radians to degrees
                scaled_position = msg.position[i] * self.scaling_factor
                goal_pos_deg = math.degrees(scaled_position)
                
                # Apply calibration offset if available and enabled
                if self.use_calibration and name in self.calibration_offsets:
                    offset = self.calibration_offsets[name]
                    goal_pos_deg += offset
                    self.get_logger().debug(f"Applied calibration offset for {name}: {offset:.1f}°")
                
                # Convert to integer for motor control
                goal_positions.append(int(goal_pos_deg))
        
        if motor_names_to_write:
            self.get_logger().debug(f"Following leader: writing positions: {goal_positions} to motors: {motor_names_to_write}")
            try:
                self.motors_bus.write("Goal_Position", goal_positions, motor_names=motor_names_to_write)
            except Exception as e:
                self.get_logger().error(f"Failed to write to follower arm motors: {e}")

    def _publish_follower_joint_states_callback(self):
        """Publish current follower arm joint states"""
        try:
            positions_deg = self.motors_bus.read("Present_Position")
            positions_rad = [math.radians(p) for p in positions_deg]
            
            # Read velocity data from motors (in RPM, convert to rad/s)
            velocities_rpm = self.motors_bus.read("Present_Speed")
            velocities_rad_s = [math.radians(v * 6) for v in velocities_rpm]  # Convert RPM to rad/s (1 RPM = 6 deg/s)

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = positions_rad
            joint_state_msg.velocity = velocities_rad_s

            self.follower_joint_state_publisher.publish(joint_state_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to read from follower arm motors: {e}")

    def set_calibration_offsets(self, offsets):
        """Set calibration offsets for joints"""
        self.calibration_offsets = offsets
        self.get_logger().info(f"Calibration offsets set: {offsets}")

    def destroy_node(self):
        self.get_logger().info("Shutting down follower arm node, disconnecting from motors.")
        if self.motors_bus and getattr(self, 'is_connected', False):
            self.motors_bus.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FollowerArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 