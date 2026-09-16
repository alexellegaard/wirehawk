import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('wirehawk_gazebo')
    pkg_prefix = get_package_prefix('wirehawk_gazebo')

    world = LaunchConfiguration('world')
    plugin_path = os.path.join(pkg_prefix, 'lib')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='20x20_3m',
                              description='World variant: 20x20_3m, 20x20_5m, 70x70_3m, 70x70_5m'),

        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -v 4 ', pkg_share, '/worlds/wirehawk_world_', world, '.sdf']}.items()
        ),

        # The Translator: Converts ROS 2 Arrays to Gazebo Double messages
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cdpr/l0@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/l1@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/l2@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/l3@std_msgs/msg/Float64]gz.msgs.Double',
                '/cdpr/t0@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t1@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t2@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t3@std_msgs/msg/Float64[gz.msgs.Double'
            ],
            output='screen'
        )
    ])
