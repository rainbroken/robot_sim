from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    share = Path(get_package_share_directory('robot_navigation'))
    nav2 = Path(get_package_share_directory('nav2_bringup'))
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f'Navigation map does not exist: {map_path}')
    data = yaml.safe_load(map_path.read_text())
    # TimerAction executes after an enclosing GroupAction may have popped its
    # launch configuration scope. Capture this value while it is available.
    autostart = LaunchConfiguration('autostart').perform(context)
    image = Path(data['image']).expanduser()
    if not image.is_absolute(): image = map_path.parent / image
    if not image.is_file(): raise RuntimeError(f'Map image missing: {image}')
    return [
        LogInfo(msg='Nav2 only: an external localization provider must publish map -> odom; '
                    'the simulator must provide odom -> base_footprint, /odom and /scan.'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'use_sim_time': True, 'yaml_filename': str(map_path)}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
             parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}], output='screen'),
        # Delay Nav2 lifecycle traffic to reduce simultaneous DDS discovery.
        # This delay does not establish or verify external localization readiness.
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav2 / 'launch/navigation_launch.py')),
            launch_arguments={'use_sim_time': 'True', 'autostart': autostart, 'use_composition': 'False',
                              'params_file': str(share / 'config/nav2.yaml')}.items())]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map', description='Occupancy map YAML; localization is external'),
                              DeclareLaunchArgument('autostart', default_value='true'),
                              OpaqueFunction(function=setup)])
