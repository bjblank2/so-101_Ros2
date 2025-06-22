import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('so_arm_feetech_interface')
    
    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(package_dir, 'config', 'leader_follower_config.yaml'),
        description='Path to the configuration file'
    )
    
    # Get the config path
    config_path = LaunchConfiguration('config_path')
    
    # Leader arm node
    leader_arm_node = Node(
        package='so_arm_feetech_interface',
        executable='leader_arm_node',
        name='leader_arm_node',
        parameters=[{'config_path': config_path}],
        output='screen',
        emulate_tty=True,
    )
    
    # Follower arm node
    follower_arm_node = Node(
        package='so_arm_feetech_interface',
        executable='follower_arm_node',
        name='follower_arm_node',
        parameters=[{'config_path': config_path}],
        output='screen',
        emulate_tty=True,
    )
    
    # Calibration node
    calibration_node = Node(
        package='so_arm_feetech_interface',
        executable='calibration_node',
        name='calibration_node',
        parameters=[{'config_path': config_path}],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_path_arg,
        leader_arm_node,
        follower_arm_node,
        calibration_node,
    ]) 