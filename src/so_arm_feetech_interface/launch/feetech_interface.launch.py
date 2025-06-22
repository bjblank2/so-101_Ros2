import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('so_arm_feetech_interface')
    config_path = os.path.join(pkg_share, 'config', 'feetech_config.yaml')

    feetech_interface_node = Node(
        package='so_arm_feetech_interface',
        executable='feetech_interface_node',
        name='feetech_hardware_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{'config_path': config_path}]
    )

    return LaunchDescription([
        feetech_interface_node
    ]) 