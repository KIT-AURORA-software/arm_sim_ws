import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('aurora_arm')
    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # Ignition Gazebo を起動
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'),

        # robot_state_publisher（URDFをPublish）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),

        # IgnitionへSpawn
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'aurora_arm', '-topic', 'robot_description'],
            output='screen')
    ])
