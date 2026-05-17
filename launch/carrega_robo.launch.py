from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-2.8')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.2')

    urdf_path = PathJoinSubstitution([
        FindPackageShare("ssc0712_t1"),
        "description",
        "robot.urdf.xacro",
    ])
    robot_urdf_final = Command(["xacro ", urdf_path])

    diff_drive_params = PathJoinSubstitution([
        FindPackageShare("ssc0712_t1"),
        "config",
        "controller_config.yaml",
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_urdf_final}],
    )

    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=["ros2", "control", "load_controller", "--set-state", "active",
             "joint_state_broadcaster"],
        shell=False,
        output="screen",
    )

    start_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_diff_drive_base_controller",
        arguments=["diff_drive_base_controller"],
        parameters=[diff_drive_params],
        output="screen",
    )

    start_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=["gripper_controller"],
        parameters=[diff_drive_params],
        output="screen",
    )

    # diff_drive_controller publica em cmd_vel_unstamped; o mission_control publica em /cmd_vel.
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[{
            "input_topic": "/cmd_vel",
            "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
        }],
        output="screen",
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ssc0712_t1"),
        "rviz",
        "rviz_config.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "prm_robot",
            "-topic", "robot_description",
            "-x", LaunchConfiguration('spawn_x'),
            "-y", LaunchConfiguration('spawn_y'),
            "-z", LaunchConfiguration('spawn_z'),
            "--ros-args", "--log-level", "warn",
        ],
        parameters=[{"use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_prm_robot",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            "/robot_cam/labels_map@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam/colored_map@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/prm_robot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
        output="screen",
    )

    odom_gt = Node(
        package="ssc0712_t1",
        executable="ground_truth_odometry",
        name="odom_gt",
        output="screen",
    )

    robo_mapper = Node(
        package="ssc0712_t1",
        executable="robo_mapper",
        name="robo_mapper",
        output="screen",
    )

    return LaunchDescription([
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        bridge,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[start_diff_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_diff_controller,
                on_exit=[start_gripper_controller],
            )
        ),
        odom_gt,
        robo_mapper,
        rviz_node,
        relay_cmd_vel,
    ])
