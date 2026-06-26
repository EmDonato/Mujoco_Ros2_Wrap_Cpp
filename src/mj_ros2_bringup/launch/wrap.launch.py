import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    config_file = os.path.join(
        get_package_share_directory('mj_ros2_bringup'),
        'config',
        'mj_ros2_params.yaml'
    )

    file_name = LaunchConfiguration('file_name').perform(context)

    parameters = [config_file]

    # Override the YAML parameter only if file_name is explicitly provided.
    if file_name != '':
        parameters.append({
            'file_name': file_name
        })

    node = Node(
        package='mujoco_ros2_wrapper',
        executable='mj_ros2_wrap',
        name='mj_ros2_wrap',
        parameters=parameters,
        output='screen',
    )

    return [node]


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'file_name',
            default_value='',
            description='Optional MuJoCo XML model file to load'
        ),

        OpaqueFunction(function=launch_setup)
    ])