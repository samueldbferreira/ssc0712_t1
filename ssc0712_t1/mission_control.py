#!/usr/bin/env python3
"""Mission Control - maquina de estados do Trabalho 1.

Implementa o ciclo:
    AGUARDANDO_COMANDO -> EXPLORANDO -> BANDEIRA_DETECTADA
        -> NAVEGANDO_PARA_BANDEIRA -> POSICIONANDO_PARA_COLETA -> CAPTURADA
    com transicao REDETECTANDO_BANDEIRA quando perde a bandeira no FOV.

Entradas:
    /flag_detection (Float32MultiArray, layout [detected, cx, cy, area])
    /scan (LaserScan, 360 samples, idx 0 = frente do robo)
    /start_mission (Bool) - opcional, sinaliza saida de AGUARDANDO

Saidas:
    /cmd_vel (Twist)
    /mission_state (String) - publicado a cada transicao
    /gripper_controller/commands (Float64MultiArray) - animacao ao capturar

Parametros:
    auto_start (bool, default True): se True, inicia EXPLORANDO apos delay.
    capture_distance (float, default 0.40): alvo de distancia frontal final
        (LIDAR) na captura. < ARM_REACH para que o braco toque o mastro.
    close_area_ratio (float, default 0.04): area ratio que dispara POSICIONANDO.
    lost_frames_to_redetect (int, default 30): N frames sem bandeira -> REDETECTANDO.
"""
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String


class State(Enum):
    AGUARDANDO_COMANDO = "AGUARDANDO_COMANDO"
    EXPLORANDO = "EXPLORANDO"
    BANDEIRA_DETECTADA = "BANDEIRA_DETECTADA"
    NAVEGANDO_PARA_BANDEIRA = "NAVEGANDO_PARA_BANDEIRA"
    REDETECTANDO_BANDEIRA = "REDETECTANDO_BANDEIRA"
    POSICIONANDO_PARA_COLETA = "POSICIONANDO_PARA_COLETA"
    CAPTURADA = "CAPTURADA"


# --- parametros de comportamento ---
V_EXPLORE = 0.35
V_NAV = 0.30
W_TURN_OBSTACLE = 0.8
W_TURN_SEARCH = 0.6

FRONT_HALF_DEG = 25
# O braco do robo (gripper) estica ~0.4m a frente do LIDAR sem que o LIDAR
# o veja (esta logo abaixo do plano de varredura). Logo, o threshold de
# desvio precisa ser > 0.4m (alcance do braco) + margem para reacao/frenagem.
# 0.70m da ~0.10m de folga ao braco quando o desvio dispara, considerando
# a aceleracao de 1.0 m/s^2 do controller.
OBSTACLE_DIST = 0.70

DET_DETECTED = 0
DET_CX = 1
DET_CY = 2
DET_AREA = 3
MIN_DETECT_AREA = 0.0005

# O braco estica ~0.4m a frente do LIDAR. Como o braco e invisivel ao LIDAR,
# durante um giro in-place o braco arrasta-se sobre obstaculos laterais ate
# que o LIDAR (no eixo) chegue a OBSTACLE_DIST. ARM_REACH e o raio (a partir
# do LIDAR) onde o braco pode tocar algo durante o giro; +0.10m de margem
# para folga ao corpo do obstaculo.
ARM_REACH = 0.50
# Arco frontal (em graus) que o braco varre durante um giro. 110° em cada
# hemisferio cobre giros tipicos de ate ~100° sem falso-positivo no hemisferio
# oposto.
ARM_SWEEP_ARC_DEG = 110
# Velocidade de re quando ambas as direcoes de giro estao bloqueadas pelo
# braco: pequena o suficiente para nao bater na traseira.
ARM_REVERSE_SPEED = -0.15
# Folga minima para a traseira do LIDAR antes de permitir recuo. Chassi
# termina ~0.21m atras do LIDAR; 0.35m da ~0.14m de folga real ao corpo.
REAR_SAFE_DIST = 0.35
# Roda externa a 0.233m do LIDAR + raio 0.10m -> superficie externa em
# 0.33m. Threshold 0.30m flagra qualquer obstaculo capaz de enganchar a
# roda (com pequeno buffer para falsos positivos amplos da janela 50°-130°).
# Quando dispara: evita rolar a roda em direcao ao obstaculo (causa de tombo).
WHEEL_HOOK_DIST = 0.30
# Cap de velocidade linear quando uma roda esta proxima de obstaculo.
# Reduz arrasto/atrito que pode escalar para enganche durante o avanco.
WHEEL_CAP_SPEED = 0.10

# --- anti-pendulacao (A: histerese temporal) ---
# Duracao do compromisso com uma direcao de giro. Quando linear.x ~ 0 e
# angular.z troca de sinal antes desse tempo, o filtro zera angular para
# o robo nao pendular. Watchdog (D) cuida se isso virar travamento.
TURN_COMMIT_DUR = 0.6

# --- wall-following hibrido ---
# Quando ha parede dentro de WALL_DETECT_RANGE a direita, a exploracao
# troca de serpentina para right-hand wall-following: mantem a parede
# entre WALL_FOLLOW_NEAR e WALL_FOLLOW_FAR (band). 5m cobre todo o lado
# direito de qualquer arena --- perimetro sempre conta como parede, e
# o wall-follow ativa desde o spawn (ataca o problema do arena_paredes
# em que o robo precisa contornar paredes longas para sair da base).
WALL_DETECT_RANGE = 5.0
WALL_FOLLOW_NEAR = 0.45
WALL_FOLLOW_FAR = 0.75

# --- watchdog de progresso (D: odometria) ---
# Progresso minimo de translacao ou rotacao por janela STUCK_TIME para
# considerar o robo "fazendo algo". Pequeno: 5cm ou 8.6° em 4s e bem
# baixo para qualquer estado ativo legitimo.
STUCK_DIST = 0.05
# Yaw line: progresso de busca em REDETECTANDO_BANDEIRA. So conta como
# progresso o yaw LIQUIDO (signed) desde o reference --- oscilacao 0↔+10°
# tem dyaw liquido ~0 e nao reseta o timer, ao contrario do abs() que
# resetava a cada flip e mascarava pendulacao.
STUCK_YAW_NET = 0.50
STUCK_TIME = 3.0
# Duracao total do recovery: 0.7s recuando + 0.8s girando.
RECOVERY_BACKUP_DUR = 0.7
RECOVERY_TURN_DUR = 0.8
RECOVERY_TOTAL = RECOVERY_BACKUP_DUR + RECOVERY_TURN_DUR
RECOVERY_BACKUP_SPEED = -0.15
RECOVERY_TURN_SPEED = 0.6


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 5.0)
        # Distancia frontal alvo na parada final (LIDAR). Mastro tem raio
        # 0.03m e o braco estica ~0.5m do LIDAR. Com 0.40m o braco pressiona
        # ~0.10m alem da superficie do mastro -> toque garantido.
        self.declare_parameter('capture_distance', 0.40)
        self.declare_parameter('close_area_ratio', 0.04)
        self.declare_parameter('lost_frames_to_redetect', 30)

        self.auto_start = self.get_parameter('auto_start').value
        self.auto_start_delay = float(self.get_parameter('auto_start_delay').value)
        self.capture_distance = float(self.get_parameter('capture_distance').value)
        self.close_area_ratio = float(self.get_parameter('close_area_ratio').value)
        self.lost_frames_to_redetect = int(self.get_parameter('lost_frames_to_redetect').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/mission_state', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10
        )

        self.create_subscription(Float32MultiArray, '/flag_detection',
                                 self.on_flag, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Bool, '/start_mission', self.on_start, 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_odom, 10)

        # Estado sensorial
        self.flag = (False, 0.0, 0.0, 0.0)
        self.lost_counter = 0
        self.last_flag_cx = 0.0   # ultimo cx valido quando bandeira estava visivel
        self.front_min = float('inf')
        # Zonas laterais (90°±40° = 50°-130° / 230°-310°): cobrem a posicao
        # angular da roda (~121° / ~239°, ja que a roda esta em x=-0.12m
        # do centro do LIDAR). Usadas para detectar enganche da roda.
        self.side_left_min = float('inf')
        self.side_right_min = float('inf')
        # Corridor check — dois limiares:
        #   path_*_blocked  (zona amarela 0.35m): vira e reduz velocidade
        #   inner_*_blocked (zona vermelha 0.24m): para completamente e vira forte
        self.path_left_blocked = False
        self.path_right_blocked = False
        self.inner_left_blocked = False
        self.inner_right_blocked = False
        # Arm sweep safety: True quando ha obstaculo dentro de ARM_REACH no
        # hemisferio para onde o braco varreria durante um giro naquela direcao.
        self.arm_swing_left_blocked = False
        self.arm_swing_right_blocked = False
        # Wheel hook safety: True quando ha obstaculo proximo da roda
        # (side_*_min < WHEEL_HOOK_DIST). Inibe motions que rolariam a roda
        # contra esse obstaculo.
        self.wheel_left_close = False
        self.wheel_right_close = False
        # Distancia minima no arco de cada hemisferio (para escolher o lado
        # com mais folga quando ambos estao bloqueados).
        self.arm_left_min = float('inf')
        self.arm_right_min = float('inf')
        # Distancia minima na traseira (idx 180 ± 30°) para decidir se da
        # para recuar quando o braco esta enganchado dos dois lados.
        self.rear_min = float('inf')
        # Latch de toque na bandeira. Uma vez setado, _position_step
        # retorna done=True imediato sem reavaliar distancia --- evita que
        # o robo "persiga" a bandeira derrubada quando o LIDAR perde a
        # referencia vertical do mastro.
        self._touch_latched = False
        # Anti-pendulacao: sinal do giro atual e quando foi adquirido.
        self._turn_sign = 0
        self._turn_sign_t = time.monotonic()
        # Watchdog: pose atual (x, y, yaw) do odom; ultima pose com progresso
        # registrado; instante; controle do recovery.
        self._cur_pose = None
        self._last_progress_pose = None
        self._last_progress_t = time.monotonic()
        self._recovery_until = 0.0
        self._recovery_dir = 1
        # FSM
        self.state = State.AGUARDANDO_COMANDO
        self._announce_state()
        self.state_entered_at = time.monotonic()
        self.start_time = time.monotonic()
        self.serpentine_phase = 0.0

        self.timer = self.create_timer(0.1, self.step)

    # ------------------------------------------------------------------
    def on_flag(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        detected = msg.data[DET_DETECTED] > 0.5 and msg.data[DET_AREA] > MIN_DETECT_AREA
        self.flag = (detected, msg.data[DET_CX], msg.data[DET_CY], msg.data[DET_AREA])
        if detected:
            self.lost_counter = 0
            self.last_flag_cx = msg.data[DET_CX]   # guarda ultimo avistamento valido
        else:
            self.lost_counter += 1

    def on_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        def window_min(center_deg, half_deg):
            lo = (center_deg - half_deg) % n
            hi = (center_deg + half_deg) % n
            if lo <= hi:
                idxs = range(lo, hi + 1)
            else:
                idxs = list(range(lo, n)) + list(range(0, hi + 1))
            vals = [msg.ranges[i] for i in idxs
                    if math.isfinite(msg.ranges[i]) and msg.ranges[i] > 0.0]
            return min(vals) if vals else float('inf')

        self.front_min = window_min(0, FRONT_HALF_DEG)
        self.rear_min = window_min(180, 30)
        # Janela alargada: roda esta em x=-0.12m do centro; quando a roda esta
        # ao lado do obstaculo, o LIDAR ve esse obstaculo a ~121° (nao a 90°).
        # 90°±40° = 50°-130° cobre toda a zona de enganche da roda.
        self.side_left_min = window_min(90, 40)
        self.side_right_min = window_min(270, 40)

        # Corridor check: projeta cada leitura frontal no eixo lateral.
        # Zona amarela 0.35m: desvia suave. Zona vermelha 0.24m (roda 0.20m + margem):
        # para completamente e vira forte antes que a roda engancha no obstaculo.
        CORRIDOR_HALF_W = 0.35
        INNER_HALF_W = 0.24
        CORRIDOR_DEPTH = 0.75
        path_left = False
        path_right = False
        inner_left = False
        inner_right = False
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = math.radians(i)
            fwd = r * math.cos(a)
            if fwd <= 0.05 or fwd > CORRIDOR_DEPTH:
                continue
            lat = r * math.sin(a)   # positivo = esquerda, negativo = direita
            if 0.0 < lat < CORRIDOR_HALF_W:
                path_left = True
                if lat < INNER_HALF_W:
                    inner_left = True
            elif -CORRIDOR_HALF_W < lat < 0.0:
                path_right = True
                if lat > -INNER_HALF_W:
                    inner_right = True
        self.path_left_blocked = path_left
        self.path_right_blocked = path_right
        self.inner_left_blocked = inner_left
        self.inner_right_blocked = inner_right

        # Arm sweep check: o braco (raio ARM_REACH a partir do LIDAR) varre
        # o hemisferio na direcao do giro. Se ha obstaculo dentro desse raio
        # no arco frontal de ARM_SWEEP_ARC_DEG, o braco vai bater/arrastar.
        # Guarda tambem o min de cada arco para escolher o lado mais folgado
        # quando ambos estao bloqueados.
        arm_left_min = float('inf')
        arm_right_min = float('inf')
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= 0.0:
                continue
            # Indice i = angulo em graus (LIDAR de 360 amostras, idx 0 = frente).
            # CCW (giro positivo): braco varre indices [0, ARM_SWEEP_ARC_DEG].
            # CW  (giro negativo): braco varre indices [360-ARM_SWEEP_ARC_DEG, 359] + 0.
            if i <= ARM_SWEEP_ARC_DEG:
                arm_left_min = min(arm_left_min, r)
            if i >= 360 - ARM_SWEEP_ARC_DEG or i == 0:
                arm_right_min = min(arm_right_min, r)
        self.arm_left_min = arm_left_min
        self.arm_right_min = arm_right_min
        self.arm_swing_left_blocked = arm_left_min < ARM_REACH
        self.arm_swing_right_blocked = arm_right_min < ARM_REACH

        # Wheel proximity: side_*_min cobre 50°-130° (e 230°-310°), zona
        # angular da roda. Obstaculo dentro de WHEEL_HOOK_DIST -> risco
        # de enganche durante avanco ou giro na direcao errada.
        self.wheel_left_close = self.side_left_min < WHEEL_HOOK_DIST
        self.wheel_right_close = self.side_right_min < WHEEL_HOOK_DIST

    def on_start(self, msg: Bool):
        if msg.data and self.state == State.AGUARDANDO_COMANDO:
            self._set_state(State.EXPLORANDO)

    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw a partir do quaternion (formula padrao para rotacao em z)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._cur_pose = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw)

    # ------------------------------------------------------------------
    def step(self):
        now = time.monotonic()

        if (self.state == State.AGUARDANDO_COMANDO and self.auto_start
                and (now - self.start_time) > self.auto_start_delay):
            self._set_state(State.EXPLORANDO)

        # --- Watchdog (D): atualiza progresso e dispara recovery se travado.
        # Em recovery, sobrescreve o twist da FSM com manobra de desencaixe.
        recovery_twist = self._watchdog_step(now)
        if recovery_twist is not None:
            twist = recovery_twist
        else:
            twist = self._fsm_step(now)

        # --- Histerese (A): suprime flips de sinal de angular.z quando o
        # robo esta praticamente parado --- impede pendulacao esquerda/direita.
        twist.angular.z = self._filter_pendulation(
            twist.linear.x, twist.angular.z, now)

        self.cmd_pub.publish(twist)

    def _fsm_step(self, now: float) -> Twist:
        twist = Twist()

        if self.state == State.AGUARDANDO_COMANDO:
            pass

        elif self.state == State.EXPLORANDO:
            twist = self._explore_step()
            if self.flag[0]:
                self._set_state(State.BANDEIRA_DETECTADA)

        elif self.state == State.BANDEIRA_DETECTADA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if (now - self.state_entered_at) > 0.5:
                self._set_state(State.NAVEGANDO_PARA_BANDEIRA)

        elif self.state == State.NAVEGANDO_PARA_BANDEIRA:
            twist = self._navigate_step()
            if self.lost_counter >= self.lost_frames_to_redetect:
                self._set_state(State.REDETECTANDO_BANDEIRA)
            elif self.flag[0] and self.flag[3] >= self.close_area_ratio:
                self._set_state(State.POSICIONANDO_PARA_COLETA)

        elif self.state == State.REDETECTANDO_BANDEIRA:
            # Gira em direcao ao ultimo cx valido: se a bandeira foi vista
            # a esquerda (cx<0), gira esquerda (+); se a direita, gira direita (-).
            preferred = -1 if self.last_flag_cx > 0 else 1
            linear, ang_mult = self._resolve_turn(preferred)
            twist.linear.x = linear
            twist.angular.z = ang_mult * W_TURN_SEARCH
            if self.flag[0]:
                self._set_state(State.NAVEGANDO_PARA_BANDEIRA)
            elif (now - self.state_entered_at) > 8.0:
                self._set_state(State.EXPLORANDO)

        elif self.state == State.POSICIONANDO_PARA_COLETA:
            twist, done = self._position_step()
            if not self.flag[0] and self.lost_counter >= self.lost_frames_to_redetect:
                self._set_state(State.REDETECTANDO_BANDEIRA)
            elif done:
                self._set_state(State.CAPTURADA)

        elif self.state == State.CAPTURADA:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            t = now - self.state_entered_at
            self._celebrate(t)

        return twist

    # ------------------------------------------------------------------
    def _resolve_turn(self, preferred: int):
        """Resolve um pedido de giro in-place levando em conta seguranca da
        roda (anti-tombo), do braco e da traseira.

        Returns (linear, angular_mult). O caller multiplica angular_mult
        pela velocidade angular que aplicaria (W_TURN_OBSTACLE / W_TURN_SEARCH).

        Cinematica diferencial:
          +omega (CCW): roda esquerda rola para tras, direita para frente.
          -omega (CW):  roda esquerda rola para frente, direita para tras.
        Logo, se a roda esquerda esta proxima de obstaculo, CW aprofunda o
        enganche -> wheel_bad. Mesma logica simetrica para a direita.

        Prioridades (wheel > arm porque tombar > raspar):
          1. Direcao livre de wheel-hook e de arm-scrape: ideal.
          2. Sem wheel-hook mas com arm-scrape: gira em meia velocidade
             (aceita raspar braco para evitar tombo).
          3. Ambas direcoes wheel-hook: recua se traseira esta livre
             (re desengancha ambas as rodas).
          4. Travado em tudo: gira meia velocidade para o lado com mais
             folga --- aceita arrasto minimo para sair.
        """
        def wheel_bad(d):
            return (d > 0 and self.wheel_right_close) or \
                   (d < 0 and self.wheel_left_close)
        def arm_bad(d):
            return (d > 0 and self.arm_swing_left_blocked) or \
                   (d < 0 and self.arm_swing_right_blocked)

        other = -preferred
        if not wheel_bad(preferred) and not arm_bad(preferred):
            return 0.0, float(preferred)
        if not wheel_bad(other) and not arm_bad(other):
            return 0.0, float(other)
        if not wheel_bad(preferred):
            return 0.0, 0.5 * float(preferred)
        if not wheel_bad(other):
            return 0.0, 0.5 * float(other)
        if self.rear_min > REAR_SAFE_DIST:
            return ARM_REVERSE_SPEED, 0.0
        least_bad = 1.0 if self.arm_left_min >= self.arm_right_min else -1.0
        return 0.0, 0.5 * least_bad

    def _wheel_cap(self, v: float) -> float:
        """Limita velocidade linear de avanco quando ha obstaculo na zona
        da roda. Reverse e mantido (desengancha)."""
        if self.wheel_left_close or self.wheel_right_close:
            return min(v, WHEEL_CAP_SPEED)
        return v

    def _pick_turn_dir(self) -> int:
        """Retorna +1 (esquerda) ou -1 (direita) baseado na fase atual da
        serpentina --- segue o sentido natural do sweep, sem preferir o lado
        mais "aberto" (heuristica de espaco vazio causava U-turn na fronteira
        entre area com obstaculos e area livre, mandando o robo de volta).
        """
        return 1 if math.sin(self.serpentine_phase * 0.7) >= 0 else -1

    def _explore_step(self) -> Twist:
        """Exploracao hibrida: serpentina em espaco aberto, wall-following
        right-hand quando ha parede a direita dentro de WALL_DETECT_RANGE.

        Wall-following cobre arenas tipo labirinto (arena_paredes)
        sistematicamente. Serpentina mantida em areas abertas para o sweep
        lateral da camera continuar pegando bandeiras fora de eixo."""
        t = Twist()
        wall_on_right = self.side_right_min < WALL_DETECT_RANGE

        if self.front_min < OBSTACLE_DIST:
            # Obstaculo a frente. Em wall-follow, preferimos girar ESQUERDA
            # para nao virar contra a parede a direita.
            pref = 1 if wall_on_right else self._pick_turn_dir()
            linear, ang_mult = self._resolve_turn(pref)
            t.linear.x = linear
            t.angular.z = ang_mult * W_TURN_OBSTACLE
        else:
            # Zona vermelha: roda quase no obstaculo — para e vira forte.
            if self.inner_left_blocked and not self.inner_right_blocked:
                t.linear.x = 0.0
                t.angular.z = -0.70
            elif self.inner_right_blocked and not self.inner_left_blocked:
                t.linear.x = 0.0
                t.angular.z = 0.70
            # Zona amarela: obstaculo entrando no corredor — reduz e desvia.
            elif self.path_left_blocked and not self.path_right_blocked:
                t.linear.x = V_EXPLORE * 0.65
                t.angular.z = -0.35
            elif self.path_right_blocked and not self.path_left_blocked:
                t.linear.x = V_EXPLORE * 0.65
                t.angular.z = 0.35
            elif self.path_left_blocked and self.path_right_blocked:
                t.linear.x = V_EXPLORE * 0.40
                t.angular.z = 0.0
            elif wall_on_right:
                # Right-hand wall following: mantem side_right_min na banda
                # [WALL_FOLLOW_NEAR, WALL_FOLLOW_FAR]. Fora da banda, corrige.
                if self.side_right_min > WALL_FOLLOW_FAR:
                    # Sem parede perto -- vira direita gentilmente para
                    # reencontrar/abracar a parede.
                    t.linear.x = V_EXPLORE * 0.8
                    t.angular.z = -0.25
                elif self.side_right_min < WALL_FOLLOW_NEAR:
                    # Muito perto da parede -- afasta virando esquerda.
                    t.linear.x = V_EXPLORE * 0.8
                    t.angular.z = 0.25
                else:
                    # Dentro da banda -- segue reto ao longo da parede.
                    t.linear.x = V_EXPLORE
                    t.angular.z = 0.0
            else:
                # Sem parede a direita: serpentina adaptativa.
                self.serpentine_phase += 0.1
                # Velocidade e amplitude escalam com folga frontal.
                if self.front_min > 3.0:
                    v = V_EXPLORE * 1.5
                    amplitude = 0.40
                elif self.front_min > 1.5:
                    v = V_EXPLORE
                    amplitude = 0.20
                else:
                    v = V_EXPLORE * 0.7
                    amplitude = 0.10

                t.linear.x = v
                t.angular.z = amplitude * math.sin(self.serpentine_phase * 0.7)
        t.linear.x = self._wheel_cap(t.linear.x)
        return t

    def _navigate_step(self) -> Twist:
        """Vai em direcao a bandeira usando cx_norm como erro de heading.
        Corridor checks (zonas amarela/vermelha) sobrescrevem o tracking
        para desviar de obstaculos laterais; se a frente esta bloqueada,
        chama _resolve_turn (zera v: braco invisivel ao LIDAR encostaria
        no obstaculo se avancasse durante a rotacao)."""
        t = Twist()
        # Se bandeira visivel usa cx atual; se ocluida usa ultimo cx valido
        # com ganho reduzido (0.4) para manter rumo sem oscilar.
        if self.flag[0]:
            cx = self.flag[1]
        else:
            cx = self.last_flag_cx * 0.4
        t.angular.z = -1.2 * cx
        v = V_NAV * max(0.0, 1.0 - abs(cx))
        # Corridor check durante navegacao: SOBRESCREVE o tracking da bandeira
        # se necessario. Zona vermelha para completamente; zona amarela desvia.
        if self.inner_left_blocked and not self.inner_right_blocked:
            t.angular.z = min(t.angular.z, -0.65)
            v = 0.0
        elif self.inner_right_blocked and not self.inner_left_blocked:
            t.angular.z = max(t.angular.z, 0.65)
            v = 0.0
        elif self.path_left_blocked and not self.path_right_blocked:
            t.angular.z = min(t.angular.z, -0.45)  # garante giro para direita
            v *= 0.65
        elif self.path_right_blocked and not self.path_left_blocked:
            t.angular.z = max(t.angular.z, 0.45)   # garante giro para esquerda
            v *= 0.65
        elif self.path_left_blocked and self.path_right_blocked:
            v *= 0.40
        if self.front_min < OBSTACLE_DIST:
            linear, ang_mult = self._resolve_turn(self._pick_turn_dir())
            v = linear
            t.angular.z = ang_mult * W_TURN_OBSTACLE

        # ROD A SAFETY ESPECIFICO DA NAVEGACAO (so apos ver bandeira):
        # quando ha obstaculo na zona da roda (50°-130° dentro de 0.30m),
        # NAO basta reduzir velocidade --- o motor continua aplicando torque
        # e a roda raspa o cilindro ate enganchar. Para totalmente e usa
        # _resolve_turn para girar na direcao que desencaixa a roda (ou
        # recua se ambas as rodas estao presas). Em EXPLORANDO mantemos
        # apenas o _wheel_cap suave para nao deixar o robo medroso.
        if self.wheel_left_close or self.wheel_right_close:
            pref = 1 if self.wheel_left_close else -1
            linear, ang_mult = self._resolve_turn(pref)
            v = linear
            t.angular.z = ang_mult * W_TURN_OBSTACLE

        t.linear.x = v
        return t

    def _position_step(self):
        """Posicionamento fino: para a `capture_distance` da bandeira, centrada.

        Uma vez perto da bandeira com ela detectada, marca _touch_latched
        e retorna done imediato --- a bandeira e dinamica (cai quando o
        braco a empurra), e sem o latch o LIDAR perde a referencia
        vertical e front_min salta para uma parede distante, fazendo o
        robo acelerar e arrastar a bandeira."""
        t = Twist()
        # Ja tocou: nao reavaliar, so parar.
        if self._touch_latched:
            return t, True

        cx = self.flag[1]
        d = self.front_min
        err = d - self.capture_distance

        # Latch: bandeira visivel pela camera E LIDAR mostra mastro dentro
        # do alcance do braco --- consideramos tocada. Sair daqui e ir
        # direto para CAPTURADA sem depender de convergencia de cx/err.
        if self.flag[0] and d < self.capture_distance + 0.10:
            self._touch_latched = True
            return t, True

        t.angular.z = -1.5 * cx
        v = max(-0.05, min(0.12, 0.5 * err))
        if abs(cx) > 0.15:
            v = 0.0
        t.linear.x = v
        # Tolerancia 0.04 garante toque mesmo no pior caso (0.40+0.04=0.44 <
        # arm_reach 0.50 -> braco ainda pressiona ~0.06m alem da superficie).
        done = abs(err) < 0.04 and abs(cx) < 0.07
        return t, done

    def _celebrate(self, t: float):
        """Anima o gripper como 'comemoracao' da captura."""
        msg = Float64MultiArray()
        open_phase = (math.sin(t * 4.0) + 1.0) * 0.5
        ext = -0.5 + 0.3 * math.sin(t * 2.0)
        left = 0.06 * open_phase
        right = -0.06 * open_phase
        msg.data = [ext, left, right]
        self.gripper_pub.publish(msg)

    # ------------------------------------------------------------------
    def _set_state(self, new_state: State):
        if new_state == self.state:
            return
        self.get_logger().info(f'[FSM] {self.state.value} -> {new_state.value}')
        self.state = new_state
        self.state_entered_at = time.monotonic()
        self._announce_state()

    def _announce_state(self):
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Anti-pendulacao (A) e watchdog de progresso (D)
    # ------------------------------------------------------------------
    def _is_active_state(self) -> bool:
        """Estados em que o robo deveria estar progredindo. Em AGUARDANDO,
        BANDEIRA_DETECTADA e CAPTURADA a inatividade e intencional."""
        return self.state in (
            State.EXPLORANDO, State.NAVEGANDO_PARA_BANDEIRA,
            State.REDETECTANDO_BANDEIRA, State.POSICIONANDO_PARA_COLETA,
        )

    def _watchdog_step(self, now: float):
        """Retorna twist de recovery se o robo esta travado, None caso
        contrario. Recovery: recua RECOVERY_BACKUP_DUR + gira por
        RECOVERY_TURN_DUR em direcao alternada (cada recovery flipa).

        Criterio de progresso:
          - EXPLORANDO / NAVEGANDO / POSICIONANDO: so translacao > STUCK_DIST.
            Sem isso o robo pode pendular eternamente (gira pra um lado,
            volta, sem sair do lugar) sem disparar recovery.
          - REDETECTANDO: rotacao pura e esperada. Aceita yaw LIQUIDO
            (signed) desde o reference --- oscilacao tem dyaw_net ~0 e
            nao falsea progresso."""
        # 1. Em recovery ativo: emite o movimento e nao atualiza progresso.
        if now < self._recovery_until:
            elapsed = RECOVERY_TOTAL - (self._recovery_until - now)
            t = Twist()
            if elapsed < RECOVERY_BACKUP_DUR and self.rear_min > REAR_SAFE_DIST:
                t.linear.x = RECOVERY_BACKUP_SPEED
                t.angular.z = 0.0
            else:
                # Fase de giro (ou backup abortado por traseira bloqueada)
                t.linear.x = 0.0
                t.angular.z = RECOVERY_TURN_SPEED * self._recovery_dir
            return t

        # 2. Fora de recovery: atualiza tracker e checa se travou.
        if not self._is_active_state() or self._cur_pose is None:
            self._last_progress_pose = self._cur_pose
            self._last_progress_t = now
            return None

        if self._last_progress_pose is None:
            self._last_progress_pose = self._cur_pose
            self._last_progress_t = now
            return None

        dx = self._cur_pose[0] - self._last_progress_pose[0]
        dy = self._cur_pose[1] - self._last_progress_pose[1]
        progressed = math.hypot(dx, dy) > STUCK_DIST

        # Em REDETECTANDO_BANDEIRA, rotacao pura conta como progresso
        # legitimo --- mas medida como yaw LIQUIDO desde o reference
        # (signed, normalizado a [-pi, pi]). Oscilacao tem net ~0 e
        # corretamente NAO conta como progresso.
        if not progressed and self.state == State.REDETECTANDO_BANDEIRA:
            dyaw_net = self._cur_pose[2] - self._last_progress_pose[2]
            while dyaw_net > math.pi:
                dyaw_net -= 2 * math.pi
            while dyaw_net < -math.pi:
                dyaw_net += 2 * math.pi
            if abs(dyaw_net) > STUCK_YAW_NET:
                progressed = True

        if progressed:
            self._last_progress_pose = self._cur_pose
            self._last_progress_t = now
            return None

        if (now - self._last_progress_t) > STUCK_TIME:
            # Inicia recovery: alterna direcao a cada acionamento.
            self._recovery_until = now + RECOVERY_TOTAL
            self._recovery_dir = -self._recovery_dir
            self._last_progress_pose = self._cur_pose
            self._last_progress_t = now
            self.get_logger().warn(
                f'[watchdog] sem progresso >{STUCK_TIME}s em {self.state.value}, '
                f'iniciando recovery (dir={self._recovery_dir})')
            t = Twist()
            t.linear.x = RECOVERY_BACKUP_SPEED
            t.angular.z = 0.0
            return t

        return None

    def _filter_pendulation(self, lin: float, ang: float, now: float) -> float:
        """Suprime flips rapidos de sinal de angular.z quando o robo esta
        praticamente parado. Movimento linear nao-trivial libera o filtro.

        Quando um flip e pedido dentro do commit window, FORCA a direcao
        comprometida (mantendo a magnitude pedida). Antes zerava --- mas
        com angular=0 e linear=0 o robo paralisava."""
        new_sign = 0
        if abs(ang) > 0.05:
            new_sign = 1 if ang > 0 else -1

        # Em movimento linear: filtro nao atua, so atualiza tracker.
        if abs(lin) > 0.05:
            self._turn_sign = new_sign
            if new_sign != 0:
                self._turn_sign_t = now
            return ang

        # Sem giro ou mesmo sinal: ok.
        if new_sign == 0 or new_sign == self._turn_sign:
            self._turn_sign = new_sign
            if new_sign != 0:
                self._turn_sign_t = now
            return ang

        # Troca de sinal pedida. Se o sinal anterior ainda esta no commit,
        # forca a direcao antiga na magnitude pedida --- robo continua
        # girando consistentemente em vez de pendular.
        if (now - self._turn_sign_t) < TURN_COMMIT_DUR and self._turn_sign != 0:
            return abs(ang) * float(self._turn_sign)

        self._turn_sign = new_sign
        self._turn_sign_t = now
        return ang


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
