import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wirehawk_gazebo')
    pkg_prefix = get_package_prefix('wirehawk_gazebo')
    control_share = get_package_share_directory('wirehawk_control')

    world = LaunchConfiguration('world')
    plugin_path = os.path.join(pkg_prefix, 'lib')

    def _templated_sdf_launch(context):
        """Template the SDF with tcp_attachment_length from the controller's
        yaml (single source of truth), so the sim's elastic band always matches
        the value the controller uses — no second place to keep in sync."""
        world_val = world.perform(context)

        tcp_att = 0.0
        yaml_path = os.path.join(control_share, 'config', f'cdpr_params_{world_val}.yaml')
        if os.path.exists(yaml_path):
            with open(yaml_path) as f:
                params = yaml.safe_load(f) or {}
            tcp_att = float(
                params.get('cdpr_node', {})
                .get('ros__parameters', {})
                .get('tcp_attachment_length', 0.0)
            )

        sdf_tpl = os.path.join(pkg_share, 'worlds', f'wirehawk_world_{world_val}.sdf')
        with open(sdf_tpl) as f:
            sdf = f.read()
        sdf = sdf.replace('@TCP_ATTACHMENT_LENGTH@', f'{tcp_att:g}')

        fd, sdf_path = tempfile.mkstemp(
            suffix='.sdf', prefix=f'wirehawk_world_{world_val}_')
        with os.fdopen(fd, 'w') as f:
            f.write(sdf)

        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'),
                             'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -v 4 ', sdf_path]}.items()
        )]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='20x20_3m',
                              description='World variant: 20x20_3m, 20x20_5m, 40x40_3m, 70x70_3m, 70x70_5m'),

        AppendEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
        # Resolve model://wirehawk_gazebo/... (label textures) against the install share dir.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share)),

        # Gazebo world: templated SDF (band length injected from the control yaml).
        OpaqueFunction(function=_templated_sdf_launch),

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
                '/cdpr/t3@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t_unc0@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t_unc1@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t_unc2@std_msgs/msg/Float64[gz.msgs.Double',
                '/cdpr/t_unc3@std_msgs/msg/Float64[gz.msgs.Double'
            ],
            output='screen'
        )
    ])
