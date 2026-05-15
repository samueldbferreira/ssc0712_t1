# ssc0712_t1 — Trabalho 1: Exploração, Detecção e Captura de Bandeira

**SSC0712 — Programação de Robôs Móveis** · ICMC/USP São Carlos

Sistema autônomo em ROS 2 que explora a arena, detecta uma bandeira por
visão computacional (câmera de segmentação semântica do Gazebo) e se
posiciona para capturá-la, com controle baseado em máquina de estados e
percepção sensorial (LIDAR, IMU, odometria).

---

## Sumário

- [Requisitos](#requisitos)
- [Compilação](#compilação)
- [Execução](#execução)
- [Mapas suportados](#mapas-suportados)
- [Arquitetura](#arquitetura)
- [Máquina de estados](#máquina-de-estados)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Pôster / Slides](#pôster--slides)

---

## Requisitos

- **ROS 2 Humble** (Ubuntu 22.04) — versão exigida; o pacote NÃO roda em
  Jazzy ou outras (substituições do launch são incompatíveis entre
  versões; ver [Ambiente de execução](#ambiente-de-execução))
- **Gazebo Fortress** (Ignition Gazebo 6)
- Python 3.10+
- Dependências resolvidas via `rosdep`: `ros_gz_bridge`, `ros_gz_sim`,
  `robot_state_publisher`, `ros2_control`, `ros2_controllers`, `xacro`,
  `rviz2`, `teleop_twist_keyboard`, `cv_bridge`, `python3-opencv`,
  `scipy`

### Ambiente de execução

Se o seu host roda outro ROS 2 (ex.: Jazzy no Ubuntu 24.04), use um
container Humble para compilar **e** executar — não basta compilar
isolado. O autor desenvolveu usando um [distrobox](https://distrobox.it/)
chamado `prm-humble` (Ubuntu 22.04) com o seguinte script auxiliar no
workspace (`~/ros2_ws/container_env.sh`):

```bash
# limpa variáveis residuais do ROS Jazzy do host
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH \
      PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH \
      ROS_PYTHON_VERSION ROS_VERSION ROS_DISTRO 2>/dev/null
# remove paths do host do PATH (rbenv/pyenv/nvm/jazzy)
# ... (omitido)
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
```

Sintoma de rodar fora do container: ao chamar `ros2 launch ...` no host
Jazzy o launch falha com algo tipo
`executable '[<launch.substitutions.text_substitution.TextSubstitution ...>]' not found on the PATH`,
porque o sistema de substituições do launch tem diferenças entre Humble
e Jazzy.

---

## Compilação

Coloque o pacote dentro de `~/ros2_ws/src/`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <URL-do-seu-fork>/ssc0712_t1.git
```

**Dentro do ambiente Humble** (host nativo Ubuntu 22.04 ou container
`prm-humble` — ver [Ambiente de execução](#ambiente-de-execução)),
instale dependências e compile:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ssc0712_t1
source install/local_setup.bash
```

---

## Execução

> ⚠ Execute também **dentro do ambiente Humble**. Se você estiver no
> host com outro ROS 2, entre no container antes (ex.:
> `distrobox enter prm-humble` e `source ~/ros2_ws/container_env.sh`).

**Launch único** que sobe simulação + robô + sensores + detector visual +
máquina de estados em um só comando:

```bash
ros2 launch ssc0712_t1 start_mission.launch.py
```

O robô parte de `AGUARDANDO_COMANDO`, espera 3 s para os sensores
estabilizarem, e segue sozinho até `CAPTURADA`.

### Trocar de mapa

```bash
ros2 launch ssc0712_t1 start_mission.launch.py world:=arena_cilindros.sdf
ros2 launch ssc0712_t1 start_mission.launch.py world:=arena_paredes.sdf
ros2 launch ssc0712_t1 start_mission.launch.py world:=empty_arena.sdf
```

O launch escolhe automaticamente o `flag_label` da bandeira-alvo e a
posição de spawn do robô de acordo com o nome do mapa (ver tabela em
[Mapas suportados](#mapas-suportados)). Para sobrescrever:

```bash
ros2 launch ssc0712_t1 start_mission.launch.py \
    world:=arena_cilindros.sdf \
    flag_label:=25 \
    spawn_x:=-7.0 spawn_y:=0.5 spawn_z:=0.2
```

### Inspecionar a missão

Tópicos publicados:

| Tópico | Tipo | Conteúdo |
|---|---|---|
| `/mission_state` | `std_msgs/String` | estado atual da FSM |
| `/flag_detection` | `std_msgs/Float32MultiArray` | `[detected, cx_norm, cy_norm, area_ratio]` |
| `/cmd_vel` | `geometry_msgs/Twist` | comando do controle |
| `/scan` | `sensor_msgs/LaserScan` | LIDAR (360 raios) |
| `/imu` | `sensor_msgs/Imu` | IMU |
| `/odom_gt` | `nav_msgs/Odometry` | odometria ground-truth |
| `/robot_cam/labels_map` | `sensor_msgs/Image` | imagem de segmentação semântica |

```bash
ros2 topic echo /mission_state
ros2 topic echo /flag_detection
```

### Controle manual (alternativo)

Em outro terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Mapas suportados

Todos com bandeira embutida. O launch escolhe `flag_label` e spawn
adequados ao nome do mapa.

| Mapa | Bandeira (label) | Layout | Spawn do robô |
|---|---|---|---|
| `arena.sdf` (default) | 40 | hexágono ±3.5 m com obstáculos cilíndricos; bandeira em (1.8, 0) | `(-2.8, 0, 0.2)` |
| `arena_cilindros.sdf` | 25 (`blue_flag`) | retângulo 18×8 m, formato CTF, obstáculos cilíndricos do time azul; alvo em (+8, 0) | `(-8.0, -0.5, 0.2)` na zona verde |
| `arena_paredes.sdf` | 25 | igual ao anterior, mas com paredes no lado azul | `(-8.0, -0.5, 0.2)` na zona verde |
| `empty_arena.sdf` | 25 | retângulo 18×8 m vazio (CTF sem obstáculos) | `(-5.5, 0, 0.2)` perto do red_base |

A `flag_deploy_zone` (círculo verde no chão) marca a área onde o time
"red" deve depositar uma bandeira capturada — também usada como spawn
por ser um lugar livre de obstáculos próximo do red_base. Nos mapas
CTF a bandeira-alvo é sempre a `blue_flag` (área do adversário).

---

## Arquitetura

```
                    ┌──────────────────┐
                    │   Gazebo Sim     │
                    │   (mundo SDF)    │
                    └────────┬─────────┘
                             │ /scan, /imu, /odom_gt,
                             │ /robot_cam/labels_map
                             ▼
        ┌────────────┐   ┌──────────────────┐
        │ flag_      │◀──│ ros_gz_bridge    │
        │ detector   │   └──────────────────┘
        └──────┬─────┘
   /flag_detection│
                 ▼
        ┌──────────────────────────────┐
        │     mission_control          │
        │  (FSM + planner reativo)     │
        └────────────┬─────────────────┘
                     │ /cmd_vel
                     ▼
              diff_drive_controller (ros2_control)
                     │
                     ▼
                 Gazebo (robô)
```

### Nodos

- **`flag_detector`** (`ssc0712_t1/flag_detector.py`): assina
  `/robot_cam/labels_map`, procura pixels com o label da bandeira-alvo
  (parâmetro ROS `flag_label`, default 25). Aceita lista (ex.:
  `flag_label:="25,40"`) para múltiplos mapas. Quando o número de
  pixels detectados ≥ `MIN_PIXELS`, publica em `/flag_detection` um
  vetor de 4 floats: `[detected, cx_norm, cy_norm, area_ratio]`.

- **`mission_control`** (`ssc0712_t1/mission_control.py`): nó principal.
  Implementa a máquina de estados, consome `/flag_detection`, `/scan`,
  `/odom_gt`, decide e publica `/cmd_vel` a 10 Hz. Também publica
  `/mission_state` em cada transição e `/gripper_controller/commands`
  na captura para animar o braço.

- **`ground_truth_odometry`**: publica odometria precisa do simulador em
  `/odom_gt`, consumida pelo `mission_control` para o bias
  anti-retorno-a-base.

- **Launch unificado** (`launch/start_mission.launch.py`):
  - Inclui `inicia_simulacao.launch.py` (Gazebo + bridges do mundo)
  - Inclui `carrega_robo.launch.py` (URDF + controllers + bridges de
    sensores) com `spawn_x/y/z` parametrizados
  - Sobe `flag_detector` 8 s depois (espera a câmera publicar)
  - Sobe `mission_control` 10 s depois (espera os controllers)
  - Auto-seleciona `flag_label` e spawn baseado no nome do mapa via
    dicionário `WORLD_PRESETS`

---

## Máquina de estados

```
                  ┌─────────────────────┐
                  │ AGUARDANDO_COMANDO  │  timer 3 s ou
                  └─────────┬───────────┘  /start_mission
                            ▼
                  ┌─────────────────────┐
           ┌─────▶│     EXPLORANDO      │
           │      │  serpentina + dodge │
           │      │  + look-around 2min │
           │      └─────────┬───────────┘
           │                │ flag detectada
           │                ▼
           │      ┌─────────────────────┐
           │      │ BANDEIRA_DETECTADA  │ (0.4 s, anúncio)
           │      └─────────┬───────────┘
           │                ▼
           │      ┌─────────────────────┐    perdeu N frames
           │      │     NAVEGANDO_      │─────────────┐
           │      │   PARA_BANDEIRA     │             │
           │      │ heading-P + dodge   │             │
           │      └─────────┬───────────┘             ▼
           │                │ área ≥ threshold    ┌───────────────┐
           │                ▼                     │ REDETECTANDO_ │
           │      ┌─────────────────────┐         │   BANDEIRA    │
           │      │  POSICIONANDO_      │         │ (gira para o  │
           │      │  PARA_COLETA        │         │  último cx)   │
           │      │ alinha cx + LIDAR   │         └──────┬────────┘
           │      └─────────┬───────────┘                │
           │                ▼                            │
           │      ┌─────────────────────┐                │
           │      │     CAPTURADA       │                │
           │      │ stop + gripper anim │                │
           │      └─────────────────────┘                │
           │                                             │
           └────────────── 8 s sem achar ────────────────┘
```

### Comportamento por estado

| Estado | Comportamento | Saída |
|---|---|---|
| `AGUARDANDO_COMANDO` | parado | timer 3 s ou `/start_mission=true` |
| `EXPLORANDO` | anda para frente com serpentina; dodge frontal/lateral via LIDAR; "look-around" de 180° a cada 2 min; soft-bias anti-retorno-a-base usando yaw da odometria | bandeira visível |
| `BANDEIRA_DETECTADA` | curto (0.4 s) para registrar a transição no log | timer |
| `NAVEGANDO_PARA_BANDEIRA` | angular proporcional ao centro-x; reduz `v` com erro de heading; LIDAR continua desviando | área ≥ `close_area_ratio` → POSICIONANDO; perdeu N frames → REDETECTANDO |
| `REDETECTANDO_BANDEIRA` | gira em direção ao último `cx` válido | viu de novo → NAVEGANDO; timeout 8 s → EXPLORANDO |
| `POSICIONANDO_PARA_COLETA` | alinha `cx` fino e aproxima até `capture_distance` (LIDAR frontal); recua se raspar lateral | distância e centralização ok |
| `CAPTURADA` | para; anima o gripper (oscila o braço e abre/fecha as garras) | final |

### Robustez (critério 3 do PDF)

- **Perda da bandeira no FOV**: estado dedicado `REDETECTANDO`, gira em
  direção ao último `cx` válido.
- **Obstáculos no caminho**: dodge LIDAR em `EXPLORANDO` e `NAVEGANDO`
  com duas zonas (path/inner) e *direction-lock* temporal de 1 s para
  evitar oscilação esquerda/direita por ruído do sensor.
- **Raspagem lateral**: janela LIDAR 90°±40° detecta enganche da roda e
  força rotação suave para o lado oposto.
- **Anti-retorno-a-base**: usando o yaw de `/odom_gt`, quando o robô
  fica orientado >107° em relação a +X (direção da bandeira-alvo nos
  mapas suportados), aplica bias angular para reorientar e reduz a
  velocidade linear até a reorientação terminar.
- **Look-around periódico**: a cada 2 min sem encontrar a bandeira,
  faz uma rotação de 180° para varrer ângulos não cobertos pela
  serpentina (só dispara se o entorno tiver folga ≥ 0.55 m em todos os
  360° para o gripper não bater).

---

## Estrutura do repositório

```
ssc0712_t1/
├── config/
│   └── controller_config.yaml      # diff_drive_controller + gripper_controller
├── description/
│   └── robot.urdf.xacro            # robô diferencial: câmera segmentação, LIDAR, IMU, gripper
├── launch/
│   ├── start_mission.launch.py     # ★ launch único da missão
│   ├── inicia_simulacao.launch.py  # sobe Gazebo + bridges do mundo
│   ├── carrega_robo.launch.py      # spawna o robô + controllers
│   └── ...                         # outros launches utilitários
├── models/                         # modelos Gazebo (ObstaculosCilindricos, Paredes, ...)
├── rviz/                           # configurações do RViz
├── ssc0712_t1/
│   ├── flag_detector.py            # ★ detector visual (label segmentation)
│   ├── mission_control.py          # ★ FSM + planner reativo
│   ├── ground_truth_odometry.py    # publica /odom_gt
│   ├── robo_mapper.py              # opcional: mapa de ocupação
│   └── ...
├── world/                          # mapas SDF (arena, arena_cilindros, arena_paredes, empty_arena)
├── package.xml
├── setup.py
└── README.md
```

---

## Pôster / Slides

Material de apresentação para a feira de extensão do ICMC:

> **TODO**: substituir pelo link do pôster ou slides (Google Drive,
> Figma, repositório etc.) — exigência do PDF do trabalho.

---

## Autor

Samuel Ferreira (samuel.assuncao@usp.br) — SSC0712, ICMC/USP, 2026.

Pacote base: <https://github.com/matheusbg8/prm_2026>.

---

## Como rodar (passo a passo, no meu setup)

> Confira o prompt do terminal: se aparecer `samueldbferreira@prm-humble`,
> você já está dentro do container — pule o passo 1.

```bash
# 1. entrar no container (PULAR se ja estiver dentro)
distrobox enter prm-humble

# 2. preparar ambiente ROS (sempre antes de rodar qualquer coisa)
source ~/ros2_ws/container_env.sh

# 3. compilar
cd ~/ros2_ws
colcon build --symlink-install --packages-select ssc0712_t1

# 4. atualizar terminal
source install/local_setup.bash

# 5. rodar
ros2 launch ssc0712_t1 start_mission.launch.py
```

Atalho equivalente (rodar **do host**, fora do container — ele entra
sozinho):

```bash
~/ros2_ws/m.sh sim
~/ros2_ws/m.sh sim world:=arena_cilindros.sdf
~/ros2_ws/m.sh kill        # mata tudo
```
