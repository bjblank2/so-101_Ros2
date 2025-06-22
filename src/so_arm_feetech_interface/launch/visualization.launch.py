import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    feetech_pkg_share = get_package_share_directory('so_arm_feetech_interface')
    description_pkg_share = get_package_share_directory('so_arm_description')
    
    # Get the config and URDF file paths
    config_path = os.path.join(feetech_pkg_share, 'config', 'feetech_config.yaml')
    urdf_file = os.path.join(description_pkg_share, 'urdf', 'so_arm.urdf')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Hardware interface node
    feetech_interface_node = Node(
        package='so_arm_feetech_interface',
        executable='feetech_interface_node',
        name='feetech_hardware_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{'config_path': config_path}]
    )
    
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file, 'r').read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(feetech_pkg_share, 'config', 'rviz_config.rviz')]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        feetech_interface_node,
        robot_state_publisher_node,
        rviz_node
    ]) 