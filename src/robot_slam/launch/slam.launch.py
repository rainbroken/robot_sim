"""SLAM only; keep baobao's existing UKF and fixed robot transforms."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('robot_slam'))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping', 'localization'):
        raise ValueError('mode must be mapping/localization')
    common = {'use_sim_time': True}
    nodes = [
        Node(package='fast_lio', executable='fastlio_mapping', name='fastlio_frontend',
             parameters=[str(share / 'config/fastlio.yaml'), common,
                         {'publish.map_en': mode == 'mapping',
                          'publish.path_en': mode == 'mapping'}],
             additional_env={'OMP_NUM_THREADS': '2', 'OMP_WAIT_POLICY': 'PASSIVE'}, output='screen'),
        Node(package='robot_slam', executable='odometry_frame_converter',
             parameters=[str(share / 'config/converter.yaml'), common], output='screen'),
    ]
    if mode == 'mapping':
        nodes += [Node(package='robot_slam', executable='loop_closure_mapper',
                       parameters=[str(share / 'config/mapping.yaml'), common],
                       remappings=[('/slam_map', '/map'), ('/slam_map_updates', '/map_updates')],
                       output='screen'),
                  Node(package='robot_slam', executable='mapping_tf_bridge',
                       parameters=[common, {'fastlio_odom_topic': '/mapping/optimized_odom',
                                            'planar_navigation_tf': True}], output='screen')]
    else:
        directory = Path(LaunchConfiguration('map_dir').perform(context)).expanduser().resolve()
        if not (directory / 'localization.pcd').is_file():
            raise RuntimeError(f'Missing localization.pcd in {directory}')
        nodes.append(Node(package='robot_slam', executable='open3d_localizer',
                          parameters=[str(share / 'config/localization.yaml'), common,
                                      {'initial_map_id': directory.name,
                                       'initial_map_path': str(directory / 'localization.pcd'),
                                       'initial_visualization_map_path': str(directory / 'full_map.pcd')}],
                          additional_env={'OMP_NUM_THREADS': '2', 'OPENBLAS_NUM_THREADS': '1'},
                          output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mode', default_value='mapping'),
                              DeclareLaunchArgument('map_dir', default_value=''),
                              OpaqueFunction(function=setup)])
