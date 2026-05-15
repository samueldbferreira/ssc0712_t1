"""Launch unico do Trabalho 1.

Sobe simulacao (Gazebo + bridges) + robo (URDF, controllers, bridges de sensor)
+ nodos da missao (detector de bandeira, maquina de estados).

Uso basico:
    ros2 launch ssc0712_t1 start_mission.launch.py

Mapas suportados (todos com bandeiras embutidas):

    arena.sdf            (default, label 40, hexagono +-3.5m, spawn em -2.8)
    arena_cilindros.sdf  (CTF, label 25 azul, arena 18x8m, spawn proximo do red_base)
    arena_paredes.sdf    (CTF, label 25 azul)
    empty_arena.sdf      (CTF, label 25 azul, sem obstaculos)

    ros2 launch ssc0712_t1 start_mission.launch.py world:=arena_cilindros.sdf

O launch escolhe automaticamente:
    - `flag_label` (id da bandeira-alvo) baseado no nome do world
    - posicao de spawn do robo (proxima do "red_base" nos mapas CTF)

Argumentos sobrescritiveis:
    world         (default: arena.sdf)
    flag_label    (default: auto - 40 em arena.sdf, 25 nos demais)
    spawn_x/y/z   (default: auto - -2.8,0,0.2 em arena.sdf, -5.5,0,0.2 demais)
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Mapa de presets por nome de world. Quem fornecer um world novo pode
# sobrescrever os defaults via args.
#
# Conferindo posicoes:
#   - arena.sdf legado: arena hexagonal +-3.5m, bandeira em (1.8, 0).
#     Spawn no lado oposto, virado para a bandeira.
#   - arena_cilindros.sdf / arena_paredes.sdf: arenas CTF 18x8m.
#     Bandeira azul (alvo) em (+8, 0). flag_deploy_zone (circulo verde)
#     em (-8, -0.5) -- e' onde o time RED entrega bandeiras capturadas.
#     Spawnamos o robo em cima dessa zona verde, ja virado pra missao.
#   - empty_arena.sdf: arena CTF sem zona de deploy; usa spawn perto
#     do red_base.
WORLD_PRESETS = {
    'arena.sdf':           {'flag_label': '40', 'spawn_x': '-2.8', 'spawn_y': '0.0',  'spawn_z': '0.2'},
    'arena_cilindros.sdf': {'flag_label': '25', 'spawn_x': '-8.0', 'spawn_y': '-0.5', 'spawn_z': '0.2'},
    'arena_paredes.sdf':   {'flag_label': '25', 'spawn_x': '-8.0', 'spawn_y': '-0.5', 'spawn_z': '0.2'},
    'empty_arena.sdf':     {'flag_label': '25', 'spawn_x': '-5.5', 'spawn_y': '0.0',  'spawn_z': '0.2'},
}


def _preset_expr(field: str, world_cfg: LaunchConfiguration) -> PythonExpression:
    """Constroi expressao Python que retorna o valor do preset para o
    campo `field` baseado no nome do world. Fallback: preset do arena.sdf."""
    presets_repr = repr({k: v[field] for k, v in WORLD_PRESETS.items()})
    default_repr = repr(WORLD_PRESETS['arena.sdf'][field])
    # ex: "{'arena.sdf': '40', ...}.get('arena_cilindros.sdf', '40')"
    return PythonExpression([
        presets_repr, ".get('", world_cfg, "', ", default_repr, ")"
    ])


def generate_launch_description():
    pkg_share = FindPackageShare("ssc0712_t1")

    # --------------------------------------------------------------
    # Argumentos
    # --------------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="arena.sdf",
        description="Nome do .sdf do mundo a carregar (em world/)",
    )
    world_cfg = LaunchConfiguration('world')

    # Defaults derivados do nome do world (sobrescritiveis na linha de comando)
    flag_label_arg = DeclareLaunchArgument(
        "flag_label",
        default_value=_preset_expr('flag_label', world_cfg),
        description="Label (segmentation) da bandeira alvo. 40=arena.sdf legado, 25=blue_flag CTF",
    )
    spawn_x_arg = DeclareLaunchArgument("spawn_x",
        default_value=_preset_expr('spawn_x', world_cfg),
        description="Spawn X do robo")
    spawn_y_arg = DeclareLaunchArgument("spawn_y",
        default_value=_preset_expr('spawn_y', world_cfg),
        description="Spawn Y do robo")
    spawn_z_arg = DeclareLaunchArgument("spawn_z",
        default_value=_preset_expr('spawn_z', world_cfg),
        description="Spawn Z do robo")

    # --------------------------------------------------------------
    # Simulacao + robo (delegados aos launch files existentes)
    # --------------------------------------------------------------
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

    # --------------------------------------------------------------
    # Nodos da missao
    # --------------------------------------------------------------
    flag_detector = TimerAction(
        period=8.0,
        actions=[Node(
            package='ssc0712_t1',
            executable='flag_detector',
            name='flag_detector',
            output='screen',
            parameters=[{
                'flag_label': LaunchConfiguration('flag_label'),
            }],
        )],
    )

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
        flag_label_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        GroupAction([
            inicia_sim,
            carrega_robo,
            flag_detector,
            mission_control,
        ]),
    ])
