import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hik_camera'), 'config', 'camera_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=params_file),
        DeclareLaunchArgument(name='camera_info_url',
                              default_value=''),
        DeclareLaunchArgument(name='use_sensor_data_qos',
                              default_value='true'),
        DeclareLaunchArgument(name='camera_serial',
                              default_value=''),
        DeclareLaunchArgument(name='frame_id',
                              default_value='camera_optical_frame'),

        Node(
            package='hik_camera',
            executable='hik_camera_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'), {
                'camera_info_url': ParameterValue(
                    LaunchConfiguration('camera_info_url'), value_type=str),
                'use_sensor_data_qos': ParameterValue(
                    LaunchConfiguration('use_sensor_data_qos'), value_type=bool),
                'camera_serial': ParameterValue(
                    LaunchConfiguration('camera_serial'), value_type=str),
                'frame_id': ParameterValue(
                    LaunchConfiguration('frame_id'), value_type=str),
            }],
        )
    ])
