"""Launch unico do Trabalho 1.

Sobe simulacao (Gazebo + bridges) + robo (URDF, controllers, bridges de sensor)
+ nodos da missao (detector de bandeira, maquina de estados).

Uso:
    ros2 launch ssc0712_t1 start_mission.launch.py
    ros2 launch ssc0712_t1 start_mission.launch.py world:=arena.sdf rviz:=false
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ssc0712_t1")

    world_arg = DeclareLaunchArgument(
        "world",
        # arena.sdf eh o unico que tem o modelo da bandeira + plugins de label
        default_value="arena.sdf",
        description="Nome do .sdf do mundo (deve conter o modelo flag com label 40)",
    )

    inicia_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "inicia_simulacao.launch.py"])
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # Sobe o robo um pouco depois do gazebo subir, para evitar race na
    # criacao da entidade no simulador.
    carrega_robo = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "carrega_robo.launch.py"])
                ),
            )
        ],
    )

    # Detector visual da bandeira (label 40 do plugin de segmentacao do Gazebo).
    # Espera os primeiros segundos para a camera comecar a publicar.
    flag_detector = TimerAction(
        period=8.0,
        actions=[Node(
            package='ssc0712_t1',
            executable='flag_detector',
            name='flag_detector',
            output='screen',
        )],
    )

    # Maquina de estados (controle da missao). Sobe depois dos controllers.
    mission_control = TimerAction(
        period=10.0,
        actions=[Node(
            package='ssc0712_t1',
            executable='mission_control',
            name='mission_control',
            output='screen',
            parameters=[{
                'auto_start': True,
                'auto_start_delay': 3.0,
            }],
        )],
    )

    return LaunchDescription([
        world_arg,
        GroupAction([inicia_sim, carrega_robo, flag_detector, mission_control]),
    ])
