import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # joy_params = os.path.join(get_package_share_directory('aurora_arm'),'config','joystick.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # teleop_node = Node(
    #     package='joy_teleop',
    #     executable='joy_teleop',
    #     name='teleop_node',
    #     parameters=[joy_params, {'use_sim_time': use_sim_time}],
    # )

    joy_to_arm_control = Node(
        package='aurora_arm',
        executable='joy_to_arm_control',
        name='joy_to_arm_control',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        # teleop_node
        joy_to_arm_control
        ])