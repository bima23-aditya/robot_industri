from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    kinematics_params = {
        'wheel_separation': 0.17,
        'wheel_radius': 0.03,
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }

    inverse_kinematics_node = Node(
        package='robin_kinematics',
        executable='inverse_kinematics_node',
        output='screen',
        parameters=[kinematics_params],
    )

    forward_kinematics_node = Node(
        package='robin_kinematics',
        executable='forward_kinematics_node',
        output='screen',
        parameters=[kinematics_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        inverse_kinematics_node,
        forward_kinematics_node,
    ])
