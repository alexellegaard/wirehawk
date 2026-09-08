import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('wirehawk_gazebo')
    pkg_prefix = get_package_prefix('wirehawk_gazebo')
    
    world_file = os.path.join(pkg_share, 'worlds', 'wirehawk_world.sdf')
    plugin_path = os.path.join(pkg_prefix, 'lib')

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
        ),
        
        # The Translator: Converts ROS 2 Arrays to Gazebo Double_V messages
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cdpr/l0@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/l1@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/l2@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/l3@std_msgs/msg/Float64]gz.msgs.Double'
            ],
            output='screen'
        )
    ])