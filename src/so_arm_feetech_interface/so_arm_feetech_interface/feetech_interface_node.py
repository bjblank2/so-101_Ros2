import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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


class FeeTechHardwareInterface(Node):
    def __init__(self):
        super().__init__('feetech_hardware_interface')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        if not config_path:
            self.get_logger().error("'config_path' parameter not set. Please provide path to config.yaml")
            return

        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            params = config['feetech_hardware_interface']['ros__parameters']

        port = params['port']
        motors = params['motors']
        self.publish_rate = params.get('publish_rate', 10.0)
        self.calibration_data = params.get('calibration')

        self.get_logger().info(f"Connecting to port '{port}'...")

        # Try to fix permissions first
        success, message = fix_serial_permissions(port)
        if not success:
            self.get_logger().warning(f"Permission fix attempt: {message}")
            self.get_logger().warning("You may need to run: sudo usermod -a -G dialout $USER && sudo chmod 666 " + port)

        bus_config = FeetechMotorsBusConfig(port=port, motors=motors)
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
        self.get_logger().info(f"Successfully connected to motors: {self.joint_names}")

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_command_subscriber = self.create_subscription(
            JointState, 'joint_commands', self._joint_command_callback, 10
        )

        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_joint_states_callback)
        
        # TODO: Add services back once service generation is working
        # self.get_servo_service = self.create_service(GetServo, 'get_servo', self._get_servo_callback)
        # self.set_servo_service = self.create_service(SetServo, 'set_servo', self._set_servo_callback)

        self.get_logger().info("FeeTech hardware interface node started.")

    def _joint_command_callback(self, msg: JointState):
        motor_names_to_write = []
        goal_positions = []

        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                motor_names_to_write.append(name)
                # Assuming the command is in radians, converting to degrees for the driver
                goal_positions.append(math.degrees(msg.position[i]))
        
        if motor_names_to_write:
            self.get_logger().debug(f"Writing positions: {goal_positions} to motors: {motor_names_to_write}")
            self.motors_bus.write("Goal_Position", goal_positions, motor_names=motor_names_to_write)

    def _publish_joint_states_callback(self):
        try:
            positions_deg = self.motors_bus.read("Present_Position")
            positions_rad = [math.radians(p) for p in positions_deg]
            
            # Read velocity data from motors (in RPM, convert to rad/s)
            velocities_rpm = self.motors_bus.read("Present_Speed")
            velocities_rad_s = [math.radians(v * 6) for v in velocities_rpm]  # Convert RPM to rad/s (1 RPM = 6 deg/s)
            
            # For simplicity, we are not reading effort yet
            # efforts_raw = self.motors_bus.read("Present_Load")

            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = positions_rad
            joint_state_msg.velocity = velocities_rad_s
            # joint_state_msg.effort = ...

            self.joint_state_publisher.publish(joint_state_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to read from motors: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down, disconnecting from motors.")
        if self.motors_bus and getattr(self, 'is_connected', False):
            # Optional: Disable torque before disconnecting
            # self.motors_bus.write("Torque_Enable", 0)
            self.motors_bus.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FeeTechHardwareInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 