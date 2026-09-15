import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('wirehawk_gazebo')
    control_pkg = get_package_share_directory('wirehawk_control')

    world = LaunchConfiguration('world')
    # cdpr_params_<world>.yaml, calibrated to the matching world's anchor layout
    params_file = [control_pkg, '/config/cdpr_params_', world, '.yaml']

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'sim.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    cdpr_node = Node(
        package='wirehawk_control',
        executable='cdpr_node',
        name='cdpr_node',
        parameters=[params_file],
        output='screen'
    )

    planner_node = Node(
        package='wirehawk_control',
        executable='trajectory_planner',
        name='trajectory_planner',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='20x20_3m',
                              description='World variant: 20x20_3m, 20x20_5m, 70x70_3m, 70x70_5m'),
        sim_launch,
        cdpr_node,
        planner_node
    ])
