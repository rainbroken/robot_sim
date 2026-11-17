"""Baobao navigation: JUNIOR custom controller and BT, external localization."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def setup(context):
    share = Path(get_package_share_directory('robot_navigation'))
    live_map_text = LaunchConfiguration('use_live_map').perform(context).lower()
    if live_map_text not in ('true', 'false'):
        raise ValueError('use_live_map must be true/false')
    map_nodes = []
    if live_map_text == 'false':
        map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
        if not map_path.is_file():
            raise RuntimeError(f'Navigation map does not exist: {map_path}')
        data = yaml.safe_load(map_path.read_text())
        image = Path(data['image']).expanduser()
        if not image.is_absolute():
            image = map_path.parent / image
        if not image.is_file():
            raise RuntimeError(f'Map image missing: {image}')
        map_nodes = [
            Node(package='nav2_map_server', executable='map_server', name='map_server',
                 parameters=[{'use_sim_time': True, 'yaml_filename': str(map_path)}], output='screen'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}], output='screen'),
        ]
    autostart_text = LaunchConfiguration('autostart').perform(context).lower()
    if autostart_text not in ('true', 'false'):
        raise ValueError('autostart must be true/false')
    params = str(Path(LaunchConfiguration('params_file').perform(context)).expanduser().resolve())
    if not Path(params).is_file():
        raise RuntimeError(f'Navigation parameters do not exist: {params}')
    servers = [('nav2_planner', 'planner_server'),
               ('nav2_controller', 'controller_server'),
               ('nav2_behaviors', 'behavior_server'),
               ('nav2_bt_navigator', 'bt_navigator')]
    nodes = [
        LogInfo(msg='Baobao differential DWB + footprint-checked clearance scoring + SmacPlanner2D; '
                    'external localization owns map -> odom.'),
    ] + map_nodes
    for package, name in servers:
        overrides = {'use_sim_time': True}
        if name == 'bt_navigator':
            overrides['default_nav_to_pose_bt_xml'] = str(
                share / 'behavior_trees/navigate_to_pose_no_backup.xml')
            overrides['default_nav_through_poses_bt_xml'] = str(
                share / 'behavior_trees/navigate_through_poses_no_backup.xml')
        nodes.append(Node(package=package, executable=name, name=name,
                          parameters=[params, overrides], output='screen',
                          additional_env={'OMP_NUM_THREADS': '2', 'OMP_WAIT_POLICY': 'PASSIVE',
                                          'OMP_DYNAMIC': 'FALSE'}))
    # Differential commands, including low speeds, pass through the existing
    # command_watchdog for localization and stale-command stops.
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[{'use_sim_time': True, 'autostart': autostart_text == 'true',
                                   'node_names': [name for _, name in servers]}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map', default_value='', description='Occupancy YAML; localization is external'),
                              DeclareLaunchArgument('params_file', default_value=str(
                                  Path(get_package_share_directory('robot_navigation')) / 'config/nav2.yaml')),
                              DeclareLaunchArgument('use_live_map', default_value='false',
                                                    description='Use the SLAM /map publisher without starting a map server'),
                              DeclareLaunchArgument('autostart', default_value='true'),
                              OpaqueFunction(function=setup)])
