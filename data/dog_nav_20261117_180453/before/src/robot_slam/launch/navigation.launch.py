from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('robot_slam'))
    nav = Path(get_package_share_directory('robot_navigation'))
    directory = Path(LaunchConfiguration('map_dir').perform(context)).expanduser().resolve()
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/slam.launch.py')),
            launch_arguments={'mode': 'localization', 'map_dir': str(directory)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav / 'launch/navigation.launch.py')),
            launch_arguments={'map': str(directory / 'occupancy.yaml'), 'autostart': 'false'}.items()),
        Node(package='robot_slam', executable='navigation_gate', parameters=[{'use_sim_time': True}], output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_dir'), OpaqueFunction(function=setup)])
