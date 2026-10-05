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

    # Low-level controller: /cmd_vel -> cable lengths. Always running.
    cdpr_node = Node(
        package='wirehawk_control',
        executable='cdpr_node',
        name='cdpr_node',
        parameters=[params_file],
        output='screen'
    )

    # Sim bridge: stands in for the EtherCAT bridge. Consumes cdpr_node's
    # cmd/motors (counts), drives the Gazebo cable lengths, and returns
    # state/motors (counts) so the controller loop closes on the sim backend.
    sim_bridge = Node(
        package='wirehawk_gazebo_bridge',
        executable='sim_bridge',
        name='sim_bridge',
        output='screen'
    )

    # NOTE: trajectory_planner and keyboard_teleop are ALTERNATIVE /cmd_vel
    # sources (goal-based vs manual) and must NOT run at the same time — the
    # planner holds its initial pose and would cancel teleop input. Launch them
    # manually:
    #   ros2 run wirehawk_control keyboard_teleop --ros-args \
    #       --params-file <install>/share/wirehawk_control/config/cdpr_params_<world>.yaml
    #   ros2 run wirehawk_control trajectory_planner --ros-args \
    #       --params-file <install>/share/wirehawk_control/config/cdpr_params_<world>.yaml

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='20x20_3m',
                              description='World variant: 20x20_3m, 20x20_5m, 40x40_3m, 70x70_3m, 70x70_5m'),
        sim_launch,
        cdpr_node,
        sim_bridge
    ])
