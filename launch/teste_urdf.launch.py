"""Utilitario: visualiza a URDF no RViz com joint_state_publisher_gui."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("ssc0712_t1"),
        "description",
        "robot.urdf.xacro",
    ])
    robot_description = Command(["xacro ", urdf_path])

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ssc0712_t1"),
        "rviz",
        "urdf.rviz",
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
