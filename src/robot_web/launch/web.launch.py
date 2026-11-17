from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {'host': '127.0.0.1', 'port': '8080',
                'map_root': str(Path.home() / '.local/share/baobao/maps'),
                'start_sim': 'false', 'scene': 'house', 'gui': 'true', 'rviz': 'false',
                'dynamic_pedestrians': 'true', 'pedestrian_count': '3', 'pedestrian_seed': '42',
                'sensor_noise_scale': '1.0', 'lidar_downsample': '5'}
    bringup = Path(get_package_share_directory('robot_bringup')) / 'launch/bringup.launch.py'
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup)),
            launch_arguments={k: LaunchConfiguration(k) for k in (
                'scene', 'gui', 'rviz', 'dynamic_pedestrians', 'pedestrian_count',
                'pedestrian_seed', 'sensor_noise_scale', 'lidar_downsample')}.items(),
            condition=IfCondition(LaunchConfiguration('start_sim'))),
        Node(package='robot_web', executable='robot_web_server', output='screen',
             sigterm_timeout='30', sigkill_timeout='5',
             parameters=[{'use_sim_time': True, 'host': LaunchConfiguration('host'),
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                          'map_root': LaunchConfiguration('map_root')}]),
    ])
