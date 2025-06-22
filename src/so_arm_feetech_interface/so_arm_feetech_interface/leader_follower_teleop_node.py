import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class LeaderFollowerTeleopNode(Node):
    def __init__(self):
        super().__init__('leader_follower_teleop_node')
        self.declare_parameter('leader_joint_state_topic', '/leader_arm/joint_states')
        self.declare_parameter('follower_command_topic', '/joint_commands')
        self.declare_parameter('joint_name_map', {})  # Optional: mapping dict

        leader_topic = self.get_parameter('leader_joint_state_topic').get_parameter_value().string_value
        follower_topic = self.get_parameter('follower_command_topic').get_parameter_value().string_value
        self.joint_name_map = self.get_parameter('joint_name_map').get_parameter_value().string_value

        if self.joint_name_map:
            import ast
            self.joint_name_map = ast.literal_eval(self.joint_name_map)
        else:
            self.joint_name_map = {}

        self.get_logger().info(f"Subscribing to leader joint states: {leader_topic}")
        self.get_logger().info(f"Publishing follower joint commands: {follower_topic}")
        if self.joint_name_map:
            self.get_logger().info(f"Using joint name mapping: {self.joint_name_map}")

        self.publisher = self.create_publisher(JointState, follower_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            leader_topic,
            self.leader_joint_state_callback,
            10
        )

    def leader_joint_state_callback(self, msg: JointState):
        # Map joint names if a mapping is provided
        if self.joint_name_map:
            mapped_names = [self.joint_name_map.get(name, name) for name in msg.name]
        else:
            mapped_names = msg.name
        out_msg = JointState()
        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.name = mapped_names
        out_msg.position = msg.position
        # Optionally, copy velocity/effort if needed
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 