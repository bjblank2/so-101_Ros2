from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_joint_state_topic',
            default_value='/leader_arm/joint_states',
            description='Topic to subscribe for leader arm joint states.'
        ),
        DeclareLaunchArgument(
            'follower_command_topic',
            default_value='/joint_commands',
            description='Topic to publish follower arm joint commands.'
        ),
        DeclareLaunchArgument(
            'joint_name_map',
            default_value='',
            description='Optional mapping from leader to follower joint names as a Python dict string.'
        ),
        Node(
            package='so_arm_feetech_interface',
            executable='leader_follower_teleop_node',
            name='leader_follower_teleop_node',
            output='screen',
            parameters=[
                {'leader_joint_state_topic': LaunchConfiguration('leader_joint_state_topic')},
                {'follower_command_topic': LaunchConfiguration('follower_command_topic')},
                {'joint_name_map': LaunchConfiguration('joint_name_map')},
            ]
        )
    ]) 