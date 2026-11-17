"""Select a local Gazebo world while retaining the differential robot model."""
from pathlib import Path
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, SetLaunchConfiguration)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _validate_noise_scale(context):
    value = float(LaunchConfiguration('sensor_noise_scale').perform(context))
    if not math.isfinite(value) or value < 0:
        raise ValueError('sensor_noise_scale must be finite and non-negative')
    downsample = int(LaunchConfiguration('lidar_downsample').perform(context))
    if not 1 <= downsample <= 100:
        raise ValueError('lidar_downsample must be in 1..100')
    return []


def _robot_nodes(context):
    model = Path(get_package_share_directory('robot_description'))
    description = xacro.process_file(str(model / 'xacro/baobaowan_B_800mm.urdf.xacro'), mappings={
        'gazebo': 'true',
        'joint_damping': '0.001',
        'lidar_downsample': LaunchConfiguration('lidar_downsample').perform(context),
        'sensor_noise_scale': LaunchConfiguration('sensor_noise_scale').perform(context),
    }).toxml()
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen')]


def _prepare_pedestrians(context):
    enabled = LaunchConfiguration('dynamic_pedestrians').perform(context).lower()
    if enabled not in ('true', 'false', '1', '0'):
        raise ValueError('dynamic_pedestrians must be true or false')
    count = int(LaunchConfiguration('pedestrian_count').perform(context))
    seed = int(LaunchConfiguration('pedestrian_seed').perform(context))
    speed = float(LaunchConfiguration('pedestrian_max_speed').perform(context))
    if not 0 <= count <= 20 or not 0 <= seed <= 4294967295:
        raise ValueError('pedestrian_count must be 0..20; pedestrian_seed must be uint32')
    if not math.isfinite(speed) or not 0 < speed <= 2.0:
        raise ValueError('pedestrian_max_speed must be finite and in (0, 2] m/s')
    tree = ET.parse(LaunchConfiguration('world').perform(context))
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError('Expected an SDF world for dynamic pedestrians')
    # 1 ms divides the nominal 200 Hz IMU period exactly. A 2 ms world step
    # quantizes Gazebo Classic's 200 Hz request to ~166.7 Hz.
    physics = world.find('physics')
    if physics is None: physics = ET.SubElement(world, 'physics', name='ode', type='ode')
    if physics.get('type', 'ode') != 'ode':
        raise ValueError('The MID360 multiray plugin requires an ODE physics world')
    for key, value in [('max_step_size', '0.001'), ('real_time_update_rate', '1000')]:
        element = physics.find(key)
        if element is None: element = ET.SubElement(physics, key)
        element.text = value
    if enabled in ('true', '1'):
        library = Path(get_package_prefix('robot_sim')) / 'lib/librobot_sim_random_pedestrians.so'
        if not library.is_file():
            raise RuntimeError('Build robot_sim before enabling dynamic pedestrians')
        plugin = ET.SubElement(world, 'plugin', name='random_pedestrians', filename=str(library))
        for key, value in [('count', count), ('seed', seed), ('max_speed', speed)]:
            ET.SubElement(plugin, key).text = str(value)
    # Diagnostic ground truth ONLY. No mapping, filter or navigation node consumes it.
    state = ET.SubElement(world, 'plugin', name='gazebo_state', filename='libgazebo_ros_state.so')
    ET.SubElement(ET.SubElement(state, 'ros'), 'namespace').text = '/gazebo'
    ET.SubElement(state, 'update_rate').text = '20.0'
    # Keep shipped assets intact; each launch owns and removes its generated world.
    temporary = tempfile.TemporaryDirectory(prefix='robot_sim_pedestrians_')
    path = Path(temporary.name) / 'scene.world'
    tree.write(path, encoding='utf-8', xml_declaration=True)

    def cleanup(_context):
        temporary.cleanup()
        return []

    return [SetLaunchConfiguration('simulation_world', str(path)),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]


def generate_launch_description():
    sim = Path(get_package_share_directory('robot_sim'))
    model = Path(get_package_share_directory('robot_description'))
    gazebo = Path(get_package_share_directory('gazebo_ros'))
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # gazebo_ros spawn converts package:// meshes to model://. Its search
        # path must contain the directory containing robot_description.
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', str(model.parent) + os.pathsep + os.environ.get('GAZEBO_MODEL_PATH', '')),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
            str(Path(get_package_prefix('robot_sim')) / 'lib') + os.pathsep +
            str(Path(get_package_prefix('mid360_simulation')) / 'lib') + os.pathsep +
            os.environ.get('GAZEBO_PLUGIN_PATH', '')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('sensor_noise_scale', default_value='1.0',
                             description='Gaussian standard deviation multiplier: 0=off, 1=default'),
        DeclareLaunchArgument('lidar_downsample', default_value='5', description='1=200k rays/s; 5=40k rays/s, CSV advances at the same nominal rate'),
        OpaqueFunction(function=_validate_noise_scale),
        DeclareLaunchArgument('world', default_value=str(sim / 'worlds/interview_office.world')),
        DeclareLaunchArgument('dynamic_pedestrians', default_value='true'),
        DeclareLaunchArgument('pedestrian_count', default_value='3'),
        DeclareLaunchArgument('pedestrian_seed', default_value='42'),
        DeclareLaunchArgument('pedestrian_max_speed', default_value='0.7', description='m/s, (0, 2]'),
        OpaqueFunction(function=_prepare_pedestrians),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch/gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('simulation_world'), 'gui': LaunchConfiguration('gui'),
                              'server_required': 'true',
                              'params_file': str(sim / 'config/gazebo.yaml')}.items()),
        OpaqueFunction(function=_robot_nodes),
        Node(package='robot_localization', executable='ukf_node', name='ukf_filter_node',
             parameters=[str(sim / 'config/ukf.yaml'), {'use_sim_time': True}],
             remappings=[('/odometry/filtered', '/odom')], output='screen'),
        Node(package='robot_sim', executable='command_watchdog',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='robot_sim', executable='cloud_to_scan',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', 'robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.003']),
    ])
