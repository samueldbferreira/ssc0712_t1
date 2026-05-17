from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default=''),
        ])
    }

    world_file_arg = DeclareLaunchArgument(
        'world', default_value='arena_cilindros.sdf',
        description='Nome do arquivo .sdf do mundo (em world/)',
    )

    pkg_share = FindPackageShare("ssc0712_t1").find("ssc0712_t1")
    world_path = PathJoinSubstitution([
        pkg_share, "world", LaunchConfiguration('world'),
    ])

    gazebo = ExecuteProcess(
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', '3', world_path],
        output='screen',
        additional_env=gz_env,
        shell=False,
    )

    gz_models_path = ":".join([pkg_share, os.path.join(pkg_share, "models")])
    gz_set_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gz_models_path,
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_world",
        arguments=[
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([
        world_file_arg,
        gz_set_env,
        bridge,
        gazebo,
    ])
