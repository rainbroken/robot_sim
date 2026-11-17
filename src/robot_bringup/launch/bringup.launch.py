from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    sim = Path(get_package_share_directory('robot_sim'))
    scene = LaunchConfiguration('scene').perform(context)
    if scene not in ('house', 'office'):
        raise ValueError('scene must be house/office')
    world = LaunchConfiguration('world').perform(context) or str(sim / 'worlds' / (
        'family_house.world' if scene == 'house' else 'interview_office.world'))
    args = {k: LaunchConfiguration(k) for k in ['gui', 'dynamic_pedestrians', 'pedestrian_count',
            'pedestrian_seed', 'pedestrian_max_speed', 'lidar_downsample', 'sensor_noise_scale']}
    args['world'] = world
    nodes = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim / 'launch/sim.launch.py')),
                                     launch_arguments=args.items())]
    nodes.append(Node(package='rviz2', executable='rviz2',
                      arguments=['-d', str(sim / 'rviz/simulation.rviz')],
                      parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))))
    return nodes


def generate_launch_description():
    defaults = {'scene': 'house', 'world': '', 'gui': 'true', 'rviz': 'true',
                'dynamic_pedestrians': 'true', 'pedestrian_count': '3', 'pedestrian_seed': '42',
                'pedestrian_max_speed': '0.7', 'lidar_downsample': '5', 'sensor_noise_scale': '1.0'}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=setup)])
