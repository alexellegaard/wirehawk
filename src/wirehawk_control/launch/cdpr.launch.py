import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('wirehawk_gazebo')
    control_pkg = get_package_share_directory('wirehawk_control')

    params_file = os.path.join(control_pkg, 'config', 'cdpr_params.yaml')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')
        )
    )

    cdpr_node = Node(
        package='wirehawk_control',
        executable='cdpr_node',
        name='cdpr_node',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        cdpr_node
    ])