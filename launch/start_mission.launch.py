"""Launch unico do Trabalho 1: sobe simulacao + robo + detector + FSM.

Uso: ros2 launch ssc0712_t1 start_mission.launch.py [world:=<arquivo.sdf>]
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Defaults por mapa (sobrescritiveis via args). flag_label e o id da label
# de segmentacao da bandeira-alvo; spawn_* fica em area livre proxima da
# base "red" nos mapas CTF, ja virado para a bandeira.
WORLD_PRESETS = {
    'arena.sdf':           {'flag_label': '40', 'spawn_x': '-2.8', 'spawn_y': '0.0',  'spawn_z': '0.2'},
    'arena_cilindros.sdf': {'flag_label': '25', 'spawn_x': '-8.0', 'spawn_y': '-0.5', 'spawn_z': '0.2'},
    'arena_paredes.sdf':   {'flag_label': '25', 'spawn_x': '-8.0', 'spawn_y': '-0.5', 'spawn_z': '0.2'},
    'empty_arena.sdf':     {'flag_label': '25', 'spawn_x': '-5.5', 'spawn_y': '0.0',  'spawn_z': '0.2'},
}


def _preset_expr(field, world_cfg):
    presets_repr = repr({k: v[field] for k, v in WORLD_PRESETS.items()})
    default_repr = repr(WORLD_PRESETS['arena.sdf'][field])
    return PythonExpression([
        presets_repr, ".get('", world_cfg, "', ", default_repr, ")"
    ])


def generate_launch_description():
    pkg_share = FindPackageShare("ssc0712_t1")

    world_arg = DeclareLaunchArgument(
        "world", default_value="arena.sdf",
        description="Nome do .sdf do mundo (em world/)",
    )
    world_cfg = LaunchConfiguration('world')

    flag_label_arg = DeclareLaunchArgument(
        "flag_label", default_value=_preset_expr('flag_label', world_cfg),
        description="Label da bandeira-alvo")
    spawn_x_arg = DeclareLaunchArgument(
        "spawn_x", default_value=_preset_expr('spawn_x', world_cfg))
    spawn_y_arg = DeclareLaunchArgument(
        "spawn_y", default_value=_preset_expr('spawn_y', world_cfg))
    spawn_z_arg = DeclareLaunchArgument(
        "spawn_z", default_value=_preset_expr('spawn_z', world_cfg))

    inicia_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "inicia_simulacao.launch.py"])
        ),
        launch_arguments={"world": world_cfg}.items(),
    )

    carrega_robo = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_share, "launch", "carrega_robo.launch.py"])
                ),
                launch_arguments={
                    'spawn_x': LaunchConfiguration('spawn_x'),
                    'spawn_y': LaunchConfiguration('spawn_y'),
                    'spawn_z': LaunchConfiguration('spawn_z'),
                }.items(),
            )
        ],
    )

    flag_detector = TimerAction(
        period=8.0,
        actions=[Node(
            package='ssc0712_t1',
            executable='flag_detector',
            name='flag_detector',
            output='screen',
            parameters=[{'flag_label': LaunchConfiguration('flag_label')}],
        )],
    )

    mission_control = TimerAction(
        period=10.0,
        actions=[Node(
            package='ssc0712_t1',
            executable='mission_control',
            name='mission_control',
            output='screen',
        )],
    )

    # Ignition Gazebo + controllers deixam processos zumbis ao matar com
    # Ctrl+C. Mata antes de subir, senao a proxima execucao trava.
    cleanup_zombies = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f '
             '"ign|gz |gz-|ruby|gazebo|rviz|robot_state_publisher|'
             'spawner|controller_manager|flag_detector|mission_control|'
             'ros_gz_bridge|parameter_bridge|ground_truth|robo_mapper" '
             '|| true ; sleep 2'],
        output='log',
    )

    start_after_cleanup = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cleanup_zombies,
            on_exit=[
                GroupAction([
                    inicia_sim,
                    carrega_robo,
                    flag_detector,
                    mission_control,
                ]),
            ],
        )
    )

    return LaunchDescription([
        world_arg,
        flag_label_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        cleanup_zombies,
        start_after_cleanup,
    ])
