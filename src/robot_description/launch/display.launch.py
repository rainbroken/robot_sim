from pathlib import Path

import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_nodes(context, share):
    model = Path(LaunchConfiguration('model').perform(context)).expanduser()
    # Relative model paths refer to this installed description package.
    if not model.is_absolute():
        model = share / model
    if not model.is_file():
        raise FileNotFoundError(f'Robot description does not exist: {model}')

    if model.suffix == '.xacro':
        description = xacro.process_file(str(model)).toxml()
    elif model.suffix == '.urdf':
        description = model.read_text(encoding='utf-8-sig')
    else:
        raise ValueError('model must be a .urdf or .xacro file')

    sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    nodes = [Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(description, value_type=str),
            'use_sim_time': sim_time,
        }],
    )]

    # A preview publisher is optional so it does not compete with real encoders.
    if LaunchConfiguration('publish_joint_states').perform(context).lower() == 'true':
        nodes.extend([
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                condition=IfCondition(LaunchConfiguration('gui')),
                parameters=[{'use_sim_time': sim_time}],
                output='screen',
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                condition=UnlessCondition(LaunchConfiguration('gui')),
                parameters=[{'use_sim_time': sim_time}],
                output='screen',
            ),
        ])

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(share / 'rviz' / 'display.rviz')],
        parameters=[{'use_sim_time': sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    ))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('robot_description'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='xacro/baobaowan_B_800mm.urdf.xacro',
            description='Package-relative or absolute path to a URDF/Xacro file',
        ),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='true', choices=['true', 'false'],
            description='Disable when real hardware already publishes /joint_states',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_load_nodes, kwargs={'share': share}),
    ])
