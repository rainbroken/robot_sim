"""One entry point for Gazebo mapping or localized navigation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('robot_slam'))
    bringup = Path(get_package_share_directory('robot_bringup'))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'navigation'):
        raise ValueError('mode must be mapping/navigation')
    args = {k: LaunchConfiguration(k) for k in ('scene', 'gui', 'dynamic_pedestrians',
            'pedestrian_count', 'pedestrian_seed', 'sensor_noise_scale', 'lidar_downsample')}
    args['rviz'] = 'false'
    backend = 'slam.launch.py' if mode == 'mapping' else 'navigation.launch.py'
    return [
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/bringup.launch.py')),
            launch_arguments=args.items())]),
        GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / backend)),
            launch_arguments={'map_dir': LaunchConfiguration('map_dir')}.items())]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share / 'rviz/slam.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    defaults = {'mode': 'mapping', 'map_dir': '', 'scene': 'house', 'gui': 'true',
                'rviz': 'true', 'dynamic_pedestrians': 'true', 'pedestrian_count': '3',
                'pedestrian_seed': '42', 'sensor_noise_scale': '1.0', 'lidar_downsample': '5'}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
