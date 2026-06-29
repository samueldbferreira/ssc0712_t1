# Captura da Bandeira (Trabalhos 1 e 2)

**SSC0712 — Programação de Robôs Móveis** · ICMC/USP São Carlos

Sistema autônomo em ROS 2 que explora a arena, detecta a bandeira inimiga por
visão computacional (câmera de segmentação semântica do Gazebo), navega até
ela, **captura com o manipulador (garra), levanta, transporta de volta à base
e deposita** — tudo orquestrado por uma máquina de estados.

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
sozinho: localiza e captura a bandeira, retorna à base e deposita (`CONCLUIDO`).

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
  `/flag_detection`, `/scan`, `/odom_gt`, publica `/cmd_vel`,
  `/mission_state` e `/gripper_controller/commands` (garra). Fluxo do loop:
  watchdog → FSM → filtro anti-pendulação → publish.

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
│  │     CAPTURANDO       │ fecha + eleva  │
│  └──────────┬───────────┘                │
│             ▼                            │
│  ┌──────────────────────┐                │
│  │   RETORNANDO_BASE    │ odom + desvio  │
│  └──────────┬───────────┘                │
│             ▼                            │
│  ┌──────────────────────┐                │
│  │     DEPOSITANDO      │ abaixa + solta │
│  └──────────┬───────────┘                │
│             ▼                            │
│  ┌──────────────────────┐                │
│  │      CONCLUIDO       │                │
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
| `POSICIONANDO_PARA_COLETA` | abre a garra; alinha girando no lugar e só então faz creep reto (anti-derrubada); latch quando o mastro está na zona de pega | latch / done → CAPTURANDO |
| `CAPTURANDO` | parado; fecha as garras no mastro (~1 s) e eleva a haste | timer → RETORNANDO_BASE |
| `RETORNANDO_BASE` | recolhe a bandeira no alto (`ARM_TUCK`) e volta à **pose inicial salva** (`/odom_gt`) reusando a mesma navegação+desvio da ida; cone central do LIDAR mascarado p/ ignorar a bandeira carregada | dist < 0.5 m → DEPOSITANDO |
| `DEPOSITANDO` | abaixa a haste, abre as garras (solta) e recua | timer → CONCLUIDO |
| `CONCLUIDO` | parado; garra em repouso | final |

## Robustez

- **Perda da bandeira**: estado `REDETECTANDO` gira em direção ao último
  `cx` válido com timeout de 8 s para `EXPLORANDO`.
- **Obstáculos frontais**: `_resolve_turn` escolhe direção de giro
  respeitando segurança do braço (alcance 0.5 m), das rodas (cinemática
  diferencial) e folga traseira para recuo.
- **Obstáculos no corredor**: projeção das leituras LIDAR em (x, y) e duas
  zonas (path 0.35 m, inner 0.24 m) para reduzir velocidade ou parar.
- **Enganche da roda**: janelas LIDAR 50°–130° (esquerda) e 230°–310° (direita)
  detectam obstáculo próximo da roda (< 0.30 m). Em `NAVEGANDO` força override
  via `_resolve_turn` para evitar tombo.
- **Anti-pendulação**: histerese temporal de 0.6 s no sinal de `angular.z`
  quando o robô está parado.
- **Watchdog**: detecta travamento via `/odom_gt`. Sem progresso em 3 s,
  dispara recovery (recua 0.7 s + gira 0.8 s, direção alternada). Aceita giro
  líquido como progresso em `REDETECTANDO` e `RETORNANDO_BASE` (viradas no lugar).
- **Alinhamento da garra**: o detector estima o `cx` pelo terço inferior da
  máscara (só o mastro; o painel fica no alto e desviaria o centroide), e a
  aproximação alinha girando antes de encostar — para o mastro entrar entre as
  pinças sem ser derrubado.
- **Transporte sem esbarrar**: a bandeira é recolhida no alto (`ARM_TUCK`,
  quase vertical) durante o retorno, tirando-a da frente do robô; o cone
  central do LIDAR é mascarado para não confundir a bandeira com obstáculo.
- **Retorno sem mapa**: a pose inicial (centro da base) é salva no primeiro
  `/odom_gt` e usada como alvo de `RETORNANDO_BASE`, reusando a mesma
  navegação reativa que leva até a bandeira.

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

Samuel Ferreira — 12543565
Pacote base: <https://github.com/matheusbg8/prm_2026>.
