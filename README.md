# ssc0712_t1 — Trabalho 1: Exploração, Detecção e Captura de Bandeira

**SSC0712 — Programação de Robôs Móveis** · ICMC/USP São Carlos

Sistema autônomo em ROS 2 que explora a arena, detecta uma bandeira por
visão computacional (câmera de segmentação semântica do Gazebo) e se
posiciona para capturá-la, com controle baseado em máquina de estados.

---

## Requisitos

- **ROS 2 Humble** (Ubuntu 22.04)
- **Gazebo Fortress** (Ignition Gazebo 6)
- Python 3.10+
- Dependências resolvidas via `rosdep`: `ros_gz_bridge`, `robot_state_publisher`,
  `ros2_control`, `ros2_controllers`, `xacro`, `rviz2`, `cv_bridge`, `scipy`

## Compilação

```bash
cd ~/ros2_ws/src
git clone https://github.com/samueldbferreira/ssc0712_t1.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ssc0712_t1
source install/local_setup.bash
```

## Execução

```bash
ros2 launch ssc0712_t1 start_mission.launch.py
```

Sobe simulação + robô + sensores + detector visual + máquina de estados em
um único comando. O robô parte de `AGUARDANDO_COMANDO`, espera 5 s e segue
sozinho até `CAPTURADA`.

Trocar de mapa:

```bash
ros2 launch ssc0712_t1 start_mission.launch.py world:=arena_paredes.sdf
```

O launch escolhe automaticamente `flag_label` e posição de spawn pelo nome
do mapa. Sobrescritíveis via argumentos do launch (`flag_label:=`,
`spawn_x:=`, `spawn_y:=`, `spawn_z:=`).

### Inspecionar a missão

| Tópico | Tipo | Conteúdo |
|---|---|---|
| `/mission_state` | `std_msgs/String` | estado atual da FSM |
| `/flag_detection` | `std_msgs/Float32MultiArray` | `[detected, cx, cy, area_ratio]` |
| `/cmd_vel` | `geometry_msgs/Twist` | comando do controle |
| `/scan` | `sensor_msgs/LaserScan` | LIDAR |
| `/odom_gt` | `nav_msgs/Odometry` | odometria ground-truth |

```bash
ros2 topic echo /mission_state
```

## Mapas suportados

| Mapa | Bandeira (label) | Spawn |
|---|---|---|
| `arena.sdf` (default) | 40 | `(-2.8, 0, 0.2)` |
| `arena_cilindros.sdf` | 25 | `(-8.0, -0.5, 0.2)` |
| `arena_paredes.sdf` | 25 | `(-8.0, -0.5, 0.2)` |
| `empty_arena.sdf` | 25 | `(-5.5, 0, 0.2)` |

## Arquitetura

```
   Gazebo Sim ──► /scan, /odom_gt, /robot_cam/labels_map
                              │
                              ▼
                       flag_detector ──► /flag_detection
                                              │
                                              ▼
                                      mission_control
                                              │ /cmd_vel
                                              ▼
                              diff_drive_controller (ros2_control)
```

### Nodos

- **`flag_detector`**: assina `/robot_cam/labels_map`, procura pixels com o
  label da bandeira-alvo (parâmetro `flag_label`), publica
  `/flag_detection` com `[detected, cx_norm, cy_norm, area_ratio]`.

- **`mission_control`**: nó principal. FSM a 10 Hz, consome
  `/flag_detection`, `/scan`, `/odom_gt`, publica `/cmd_vel` e
  `/mission_state`. Fluxo do loop: watchdog → FSM → filtro anti-pendulação
  → publish.

- **`ground_truth_odometry`**: republica a pose do simulador como
  `/odom_gt` e TF `odom_gt → base_link`.

- **`robo_mapper`**: TF estático `map → odom_gt` e `OccupancyGrid` marcando
  células visitadas (visualização no RViz).

## Máquina de estados

```
   ┌──────────────────────┐  timer 5 s ou
   │  AGUARDANDO_COMANDO  │  /start_mission
   └──────────┬───────────┘
              ▼
   ┌──────────────────────┐
┌──│      EXPLORANDO      │
│  └──────────┬───────────┘
│             │ bandeira detectada
│             ▼
│  ┌──────────────────────┐
│  │ BANDEIRA_DETECTADA   │ pausa 0.5 s
│  └──────────┬───────────┘
│             ▼
│  ┌──────────────────────┐  perdeu N frames
│  │ NAVEGANDO_PARA_      │──────────────────┐
│  │   BANDEIRA           │                  ▼
│  └──────────┬───────────┘     ┌──────────────────────┐
│             │ área ≥ thresh   │ REDETECTANDO_        │
│             ▼                 │   BANDEIRA           │
│  ┌──────────────────────┐     └──────────┬───────────┘
│  │ POSICIONANDO_PARA_   │                │
│  │   COLETA             │                │
│  └──────────┬───────────┘                │
│             ▼                            │
│  ┌──────────────────────┐                │
│  │     CAPTURADA        │                │
│  └──────────────────────┘                │
│                                          │
└────────── 8 s sem achar ─────────────────┘
```

| Estado | Comportamento | Saída |
|---|---|---|
| `AGUARDANDO_COMANDO` | parado | timer 5 s ou `/start_mission=true` |
| `EXPLORANDO` | wall-following à direita se houver parede em < 5 m, senão serpentina; corridor checks para desvios; `_resolve_turn` quando a frente bloqueia | bandeira detectada |
| `BANDEIRA_DETECTADA` | parado 0.5 s para registrar a transição | timer |
| `NAVEGANDO_PARA_BANDEIRA` | heading-P em `cx`; reduz `v` com erro; override total se uma roda fica perto de obstáculo (50°–130°, < 0.30 m) | `area ≥ 0.04` → POSICIONANDO; perdeu 30 frames → REDETECTANDO |
| `REDETECTANDO_BANDEIRA` | gira em direção ao último `cx` válido | bandeira reaparece → NAVEGANDO; 8 s → EXPLORANDO |
| `POSICIONANDO_PARA_COLETA` | alinha `cx` e aproxima até 0.40 m (LIDAR); latch de toque quando bandeira visível e LIDAR < 0.50 m | latch ou `\|err\|<0.04 ∧ \|cx\|<0.07` |
| `CAPTURADA` | parado; anima o gripper | final |

## Robustez

- **Perda da bandeira**: estado `REDETECTANDO` gira em direção ao último
  `cx` válido com timeout de 8 s para `EXPLORANDO`.
- **Obstáculos frontais**: `_resolve_turn` escolhe direção de giro
  respeitando segurança do braço (alcance 0.5 m), das rodas (cinemática
  diferencial) e folga traseira para recuo.
- **Obstáculos no corredor**: projeção das leituras LIDAR em (x, y) e duas
  zonas (path 0.35 m, inner 0.24 m) para reduzir velocidade ou parar.
- **Enganche da roda**: janela LIDAR 90° ± 40° detecta obstáculo na zona da
  roda (em 121°/239° em relação ao centro do LIDAR). Em `NAVEGANDO` força
  override via `_resolve_turn` para evitar tombo.
- **Anti-pendulação**: histerese temporal de 0.6 s no sinal de `angular.z`
  quando o robô está parado.
- **Watchdog**: detecta travamento via `/odom_gt`. Sem progresso em 3 s,
  dispara recovery (recua 0.7 s + gira 0.8 s, direção alternada).
- **Captura confiável**: latch de toque garante parada imediata quando a
  bandeira está ao alcance do braço — a bandeira é dinâmica e cai ao ser
  tocada.

## Estrutura do repositório

```
ssc0712_t1/
├── config/controller_config.yaml
├── description/robot.urdf.xacro
├── launch/
│   ├── start_mission.launch.py     # launch único da missão
│   ├── inicia_simulacao.launch.py
│   ├── carrega_robo.launch.py
│   └── teste_urdf.launch.py
├── models/
├── rviz/
├── ssc0712_t1/
│   ├── mission_control.py          # FSM + planner reativo
│   ├── flag_detector.py            # detector visual
│   ├── ground_truth_odometry.py
│   └── robo_mapper.py
├── world/
├── package.xml
├── setup.py
└── README.md
```

## Pôster / Slides

[Slides](https://canva.link/dftgeds0vxx1gd6)

## Autor

Samuel Ferreira — SSC0712, ICMC/USP, 2026.
Pacote base: <https://github.com/matheusbg8/prm_2026>.
