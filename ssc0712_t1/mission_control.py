#!/usr/bin/env python3
"""Mission Control: FSM de exploracao, deteccao e captura de bandeira.

Entradas: /flag_detection (visao), /scan (LIDAR), /odom_gt (pose), /start_mission.
Saidas:   /cmd_vel, /mission_state, /gripper_controller/commands.

Loop a 10 Hz: watchdog (recovery se travado) -> FSM (computa twist do estado
atual) -> filtro anti-pendulacao -> publish.
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
    AGUARDANDO_COMANDO = "AGUARDANDO_COMANDO"            # parado ate timer/topic
    EXPLORANDO = "EXPLORANDO"                            # busca livre pela bandeira
    BANDEIRA_DETECTADA = "BANDEIRA_DETECTADA"            # pausa breve apos avistar
    NAVEGANDO_PARA_BANDEIRA = "NAVEGANDO_PARA_BANDEIRA"  # converge para o alvo
    REDETECTANDO_BANDEIRA = "REDETECTANDO_BANDEIRA"      # gira procurando alvo perdido
    POSICIONANDO_PARA_COLETA = "POSICIONANDO_PARA_COLETA"  # aproximacao final (garra aberta)
    CAPTURANDO = "CAPTURANDO"                            # fecha a garra e eleva a bandeira
    RETORNANDO_BASE = "RETORNANDO_BASE"                  # volta a pose inicial (odom + Bug)
    DEPOSITANDO = "DEPOSITANDO"                          # abaixa, solta e recua
    CONCLUIDO = "CONCLUIDO"                              # missao completa (final)


# --- velocidades (m/s e rad/s) ---
V_EXPLORE = 0.35              # avanco em exploracao
V_NAV = 0.30                  # avanco em navegacao p/ bandeira
W_TURN_OBSTACLE = 0.8         # giro angular para desviar de obstaculo
W_TURN_SEARCH = 0.6           # giro angular em REDETECTANDO

# --- janelas LIDAR ---
FRONT_HALF_DEG = 25           # meia abertura da janela frontal (0° ± FRONT_HALF_DEG)
# Braco estica ~0.4m e e invisivel ao LIDAR; threshold > alcance + frenagem.
OBSTACLE_DIST = 0.70          # distancia frontal que dispara desvio

# --- layout do /flag_detection (4 floats) ---
DET_DETECTED = 0              # 0.0 ou 1.0
DET_CX = 1                    # centro x normalizado [-1, 1]
DET_CY = 2                    # centro y normalizado [-1, 1]
DET_AREA = 3                  # fracao da imagem ocupada pela bandeira
MIN_DETECT_AREA = 0.00013     # ~10 px em 320x240; sincronizado com MIN_PIXELS do detector
# Gates da captura --- PDF exige "de frente E na distancia":
CAPTURE_MIN_AREA = 0.08       # area visual ~0.6m, confirma que LIDAR le o mastro
# cx ja aponta para o mastro (terco inferior da mascara). Tolerancia lateral
# ~+-0.03m a 0.43m -> cx=0.05 ~= 1.7cm. Apertado p/ a garra nao raspar no mastro.
CAPTURE_CX_ALIGN = 0.04       # so faz creep reto com |cx| abaixo disso (anti-derrubada)
CAPTURE_CX_LATCH = 0.06       # orientacao p/ latch (frente folgada)
CAPTURE_CX_DONE = 0.035       # orientacao p/ done estrito (de frente preciso)
CAPTURE_ERR_DONE = 0.04       # erro de distancia maximo no done estrito

# --- captura com a garra ---
# /gripper_controller/commands = [elevacao_rad, abertura_dir, abertura_esq]
# (ordem do controller_config.yaml; bate com os exemplos numericos do PDF).
ARM_DOWN = 0.0                # haste horizontal p/ frente: altura de pega do mastro
ARM_LIFT = -0.6              # eleva a bandeira apos prender (assenta o agarre)
# No transporte recolhe bem mais (quase vertical): tira a bandeira da frente do
# robo p/ nao esbarrar em obstaculos ao contornar (senao a projecao frontal bate
# de lado e capota). A junta vai ate -1.57 (vertical).
ARM_TUCK = -1.5
GRIP_OPEN_R = -0.06          # garras abertas: o mastro entra entre as pincas
GRIP_OPEN_L = 0.06
# Mastro tem superficie em +-0.03. Fecho firme (perto do maximo, +-0.005): as
# pincas param no mastro e cravam com forca -> segura ao levantar. Foi o que
# agarrou nos testes anteriores.
GRIP_CLOSE_R = -0.005
GRIP_CLOSE_L = 0.005
CAPTURE_CLOSE_DUR = 1.0      # tempo fechando as garras antes de elevar
CAPTURE_LIFT_DUR = 2.0       # tempo total de CAPTURANDO (1.0 fechar + 1.0 elevar)

# --- retorno a base por odometria (/odom_gt): vai direto, desvia pontual ---
GOAL_RADIUS = 0.5            # chegou quando dist a pose inicial < isso
RETURN_SPEED = 0.45         # avanco no retorno (mais rapido que a navegacao)
RETURN_KP_HEADING = 1.8     # ganho de heading p/ a base
RETURN_TURN_IN_PLACE = 1.0  # |erro de heading| acima disso: gira no lugar (mira rapido)
# Bandeira carregada fica no cone central do LIDAR e poluiria as leituras
# (front/arm/corredor) -> robo "veria" obstaculo permanente. Ignora esse cone
# durante o transporte (RETORNANDO/DEPOSITANDO).
FLAG_CARRY_CONE_DEG = 14

# --- deposito da bandeira (parte do ARM_TUCK alto, entao abaixa bastante) ---
DEPOSIT_LOWER_DUR = 2.2      # abaixa a haste de volta ao chao (ainda fechada)
DEPOSIT_OPEN_DUR = 3.0       # abre as garras (solta a bandeira)
DEPOSIT_BACK_DUR = 4.0       # recua p/ deixar a bandeira no lugar
DEPOSIT_BACK_SPEED = -0.15

# --- seguranca do braco (anti-arrasto ao girar) ---
ARM_REACH = 0.50              # raio em que o braco pode tocar obstaculo (m)
ARM_SWEEP_ARC_DEG = 110       # arco frontal varrido durante um giro
ARM_REVERSE_SPEED = -0.15     # velocidade de re quando travado dos dois lados
REAR_SAFE_DIST = 0.35         # folga traseira minima para permitir recuo

# --- seguranca da roda (anti-tombo) ---
# Roda em (-0.12, ±0.20); obstaculo <0.30m em 50°-130° rola contra ela.
WHEEL_HOOK_DIST = 0.30        # distancia que considera enganche iminente
WHEEL_CAP_SPEED = 0.10        # cap de avanco quando roda esta perto de obstaculo

# --- anti-pendulacao (histerese temporal no sinal angular) ---
TURN_COMMIT_DUR = 0.6         # tempo minimo que mantem a direcao escolhida (s)

# --- wall-following hibrido ---
WALL_DETECT_RANGE = 5.0       # com parede a direita < esse valor, ativa wall-follow
WALL_FOLLOW_NEAR = 0.45       # limite inferior da banda alvo (afasta da parede)
WALL_FOLLOW_FAR = 0.75        # limite superior da banda alvo (busca a parede)

# --- watchdog de progresso (recovery automatico) ---
STUCK_DIST = 0.05             # translacao minima por janela para nao ser stuck (m)
STUCK_YAW_NET = 0.50          # giro liquido minimo em REDETECTANDO (rad)
STUCK_TIME = 3.0              # tempo sem progresso ate disparar recovery (s)
RECOVERY_BACKUP_DUR = 0.7     # duracao da fase de re
RECOVERY_TURN_DUR = 0.8       # duracao da fase de giro
RECOVERY_TOTAL = RECOVERY_BACKUP_DUR + RECOVERY_TURN_DUR
RECOVERY_BACKUP_SPEED = -0.15
RECOVERY_TURN_SPEED = 0.6


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 5.0)
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

        self.create_subscription(Float32MultiArray, '/flag_detection', self.on_flag, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(Bool, '/start_mission', self.on_start, 10)
        self.create_subscription(Odometry, '/odom_gt', self.on_odom, 10)

        self.flag = (False, 0.0, 0.0, 0.0)
        self.lost_counter = 0
        self.last_flag_cx = 0.0
        self.front_min = float('inf')
        self.side_left_min = float('inf')
        self.side_right_min = float('inf')
        self.path_left_blocked = False
        self.path_right_blocked = False
        self.inner_left_blocked = False
        self.inner_right_blocked = False
        self.arm_swing_left_blocked = False
        self.arm_swing_right_blocked = False
        self.wheel_left_close = False
        self.wheel_right_close = False
        self.arm_left_min = float('inf')
        self.arm_right_min = float('inf')
        self.rear_min = float('inf')
        self._touch_latched = False
        self._turn_sign = 0
        self._turn_sign_t = time.monotonic()
        self._cur_pose = None
        self._home_pose = None       # pose inicial salva = alvo de retorno
        self._return_dir = 1         # lado de desvio comprometido (+1 esq, -1 dir)
        self._return_dir_t = time.monotonic()
        self._return_logged = False
        self._last_progress_pose = None
        self._last_progress_t = time.monotonic()
        self._recovery_until = 0.0
        self._recovery_dir = 1
        self.state = State.AGUARDANDO_COMANDO
        self._announce_state()
        self.state_entered_at = time.monotonic()
        self.start_time = time.monotonic()
        self.serpentine_phase = 0.0

        self.timer = self.create_timer(0.1, self.step)

    def on_flag(self, msg: Float32MultiArray):
        """Atualiza estado da deteccao visual e o contador de frames perdidos."""
        if len(msg.data) < 4:
            return
        detected = msg.data[DET_DETECTED] > 0.5 and msg.data[DET_AREA] > MIN_DETECT_AREA
        self.flag = (detected, msg.data[DET_CX], msg.data[DET_CY], msg.data[DET_AREA])
        if detected:
            self.lost_counter = 0
            self.last_flag_cx = msg.data[DET_CX]
        else:
            self.lost_counter += 1

    def on_scan(self, msg: LaserScan):
        """Computa todas as zonas LIDAR usadas pelo controle: janelas frontal/
        traseira/laterais, corridor checks (path/inner), arco do braco, roda."""
        n = len(msg.ranges)
        if n == 0:
            return

        ranges = msg.ranges
        # Ao carregar a bandeira, ela fica no cone central e poluiria front/arm/
        # corredor (robo "veria" obstaculo fixo). Ignora o cone central.
        if self.state in (State.RETORNANDO_BASE, State.DEPOSITANDO):
            ranges = list(msg.ranges)
            for i in range(n):
                a = i if i <= 180 else i - 360
                if -FLAG_CARRY_CONE_DEG <= a <= FLAG_CARRY_CONE_DEG:
                    ranges[i] = float('inf')

        def window_min(center_deg, half_deg):
            lo = (center_deg - half_deg) % n
            hi = (center_deg + half_deg) % n
            if lo <= hi:
                idxs = range(lo, hi + 1)
            else:
                idxs = list(range(lo, n)) + list(range(0, hi + 1))
            vals = [ranges[i] for i in idxs
                    if math.isfinite(ranges[i]) and ranges[i] > 0.0]
            return min(vals) if vals else float('inf')

        self.front_min = window_min(0, FRONT_HALF_DEG)
        self.rear_min = window_min(180, 30)
        self.side_left_min = window_min(90, 40)
        self.side_right_min = window_min(270, 40)

        CORRIDOR_HALF_W = 0.35
        INNER_HALF_W = 0.24
        CORRIDOR_DEPTH = 0.75
        path_left = path_right = inner_left = inner_right = False
        for i in range(n):
            r = ranges[i]
            if not math.isfinite(r) or r <= 0.0:
                continue
            a = math.radians(i)
            fwd = r * math.cos(a)
            if fwd <= 0.05 or fwd > CORRIDOR_DEPTH:
                continue
            lat = r * math.sin(a)
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

        arm_left_min = float('inf')
        arm_right_min = float('inf')
        for i in range(n):
            r = ranges[i]
            if not math.isfinite(r) or r <= 0.0:
                continue
            if i <= ARM_SWEEP_ARC_DEG:
                arm_left_min = min(arm_left_min, r)
            if i >= 360 - ARM_SWEEP_ARC_DEG or i == 0:
                arm_right_min = min(arm_right_min, r)
        self.arm_left_min = arm_left_min
        self.arm_right_min = arm_right_min
        self.arm_swing_left_blocked = arm_left_min < ARM_REACH
        self.arm_swing_right_blocked = arm_right_min < ARM_REACH

        self.wheel_left_close = self.side_left_min < WHEEL_HOOK_DIST
        self.wheel_right_close = self.side_right_min < WHEEL_HOOK_DIST

    def on_start(self, msg: Bool):
        """Trigger manual de saida do AGUARDANDO via /start_mission."""
        if msg.data and self.state == State.AGUARDANDO_COMANDO:
            self._set_state(State.EXPLORANDO)

    def on_odom(self, msg: Odometry):
        """Extrai (x, y, yaw) do ground-truth para o watchdog de progresso."""
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self._cur_pose = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw)
        if self._home_pose is None:
            # robo inicia sobre a base: guarda como alvo de retorno.
            self._home_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def step(self):
        """Loop principal a 10 Hz: watchdog -> FSM -> filtro anti-pendulacao -> publish."""
        now = time.monotonic()

        if (self.state == State.AGUARDANDO_COMANDO and self.auto_start
                and (now - self.start_time) > self.auto_start_delay):
            self._set_state(State.EXPLORANDO)

        recovery_twist = self._watchdog_step(now)
        twist = recovery_twist if recovery_twist is not None else self._fsm_step(now)

        twist.angular.z = self._filter_pendulation(twist.linear.x, twist.angular.z, now)
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
            if (now - self.state_entered_at) > 0.5:
                self._set_state(State.NAVEGANDO_PARA_BANDEIRA)

        elif self.state == State.NAVEGANDO_PARA_BANDEIRA:
            twist = self._navigate_step()
            if self.lost_counter >= self.lost_frames_to_redetect:
                self._set_state(State.REDETECTANDO_BANDEIRA)
            elif self.flag[0] and self.flag[3] >= self.close_area_ratio:
                self._set_state(State.POSICIONANDO_PARA_COLETA)

        elif self.state == State.REDETECTANDO_BANDEIRA:
            preferred = -1 if self.last_flag_cx > 0 else 1
            linear, ang_mult = self._resolve_turn(preferred)
            twist.linear.x = linear
            twist.angular.z = ang_mult * W_TURN_SEARCH
            if self.flag[0]:
                self._set_state(State.NAVEGANDO_PARA_BANDEIRA)
            elif (now - self.state_entered_at) > 8.0:
                self._set_state(State.EXPLORANDO)

        elif self.state == State.POSICIONANDO_PARA_COLETA:
            self._gripper(ARM_DOWN, GRIP_OPEN_R, GRIP_OPEN_L)  # abre p/ o mastro entrar
            twist, done = self._position_step()
            if not self.flag[0] and self.lost_counter >= self.lost_frames_to_redetect:
                self._set_state(State.REDETECTANDO_BANDEIRA)
            elif done:
                self._set_state(State.CAPTURANDO)

        elif self.state == State.CAPTURANDO:
            t = now - self.state_entered_at
            elevation = ARM_DOWN if t < CAPTURE_CLOSE_DUR else ARM_LIFT
            self._gripper(elevation, GRIP_CLOSE_R, GRIP_CLOSE_L)
            if t > CAPTURE_LIFT_DUR:
                self._set_state(State.RETORNANDO_BASE)

        elif self.state == State.RETORNANDO_BASE:
            if not self._return_logged and self._cur_pose and self._home_pose:
                self.get_logger().info(
                    f'[retorno] base={self._home_pose} '
                    f'pos atual=({self._cur_pose[0]:.1f},{self._cur_pose[1]:.1f})')
                self._return_logged = True
            # recolhe a bandeira no alto (ARM_TUCK) p/ nao esbarrar nos obstaculos
            self._gripper(ARM_TUCK, GRIP_CLOSE_R, GRIP_CLOSE_L)
            # navega pela arena com o MESMO wall-following (parede a direita) da
            # busca, ate a odometria indicar chegada na base.
            if (self._cur_pose and self._home_pose
                    and math.hypot(self._home_pose[0] - self._cur_pose[0],
                                   self._home_pose[1] - self._cur_pose[1]) < GOAL_RADIUS):
                self._set_state(State.DEPOSITANDO)
            else:
                twist = self._explore_step()

        elif self.state == State.DEPOSITANDO:
            t = now - self.state_entered_at
            if t < DEPOSIT_LOWER_DUR:
                self._gripper(ARM_DOWN, GRIP_CLOSE_R, GRIP_CLOSE_L)   # abaixa ainda fechada
            else:
                self._gripper(ARM_DOWN, GRIP_OPEN_R, GRIP_OPEN_L)     # abre: solta a bandeira
                if DEPOSIT_OPEN_DUR <= t < DEPOSIT_BACK_DUR:
                    twist.linear.x = DEPOSIT_BACK_SPEED               # recua p/ deixar a bandeira
                elif t >= DEPOSIT_BACK_DUR:
                    self._set_state(State.CONCLUIDO)

        elif self.state == State.CONCLUIDO:
            self._gripper(ARM_DOWN, 0.0, 0.0)

        return twist

    def _resolve_turn(self, preferred: int):
        # +omega rola roda esquerda pra tras, direita pra frente; -omega o oposto.
        # Wheel safety > arm safety (tombar e pior que raspar).
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
        """Limita o avanco quando uma roda esta perto de obstaculo (so reduz, nao para)."""
        if self.wheel_left_close or self.wheel_right_close:
            return min(v, WHEEL_CAP_SPEED)
        return v

    def _pick_turn_dir(self) -> int:
        """Direcao de giro padrao seguindo a fase da serpentina (+1=esquerda, -1=direita)."""
        return 1 if math.sin(self.serpentine_phase * 0.7) >= 0 else -1

    def _explore_step(self) -> Twist:
        """Wall-following quando ha parede a direita < 5m; serpentina em campo aberto.
        Corridor checks (path/inner) e _resolve_turn cobrem obstaculos no caminho."""
        t = Twist()
        wall_on_right = self.side_right_min < WALL_DETECT_RANGE

        if self.front_min < OBSTACLE_DIST:
            pref = 1 if wall_on_right else self._pick_turn_dir()
            linear, ang_mult = self._resolve_turn(pref)
            t.linear.x = linear
            t.angular.z = ang_mult * W_TURN_OBSTACLE
        elif self.inner_left_blocked and not self.inner_right_blocked:
            t.angular.z = -0.70
        elif self.inner_right_blocked and not self.inner_left_blocked:
            t.angular.z = 0.70
        elif self.path_left_blocked and not self.path_right_blocked:
            t.linear.x = V_EXPLORE * 0.65
            t.angular.z = -0.35
        elif self.path_right_blocked and not self.path_left_blocked:
            t.linear.x = V_EXPLORE * 0.65
            t.angular.z = 0.35
        elif self.path_left_blocked and self.path_right_blocked:
            t.linear.x = V_EXPLORE * 0.40
        elif wall_on_right:
            if self.side_right_min > WALL_FOLLOW_FAR:
                t.linear.x = V_EXPLORE * 0.8
                t.angular.z = -0.25
            elif self.side_right_min < WALL_FOLLOW_NEAR:
                t.linear.x = V_EXPLORE * 0.8
                t.angular.z = 0.25
            else:
                t.linear.x = V_EXPLORE
        else:
            self.serpentine_phase += 0.1
            if self.front_min > 3.0:
                v, amplitude = V_EXPLORE * 1.5, 0.40
            elif self.front_min > 1.5:
                v, amplitude = V_EXPLORE, 0.20
            else:
                v, amplitude = V_EXPLORE * 0.7, 0.10
            t.linear.x = v
            t.angular.z = amplitude * math.sin(self.serpentine_phase * 0.7)
        t.linear.x = self._wheel_cap(t.linear.x)
        return t

    def _navigate_toward(self, cx: float, turn_pref=None) -> Twist:
        """Heading-P em cx + reducao de v com erro, com desvio reativo (corredor,
        frente, roda). turn_pref: lado preferido ao desviar de obstaculo frontal
        (+1 esq, -1 dir) -- no retorno aponta p/ a base; senao segue a serpentina."""
        t = Twist()
        t.angular.z = -1.2 * cx
        v = V_NAV * max(0.0, 1.0 - abs(cx))

        if self.inner_left_blocked and not self.inner_right_blocked:
            t.angular.z = min(t.angular.z, -0.65)
            v = 0.0
        elif self.inner_right_blocked and not self.inner_left_blocked:
            t.angular.z = max(t.angular.z, 0.65)
            v = 0.0
        elif self.path_left_blocked and not self.path_right_blocked:
            t.angular.z = min(t.angular.z, -0.45)
            v *= 0.65
        elif self.path_right_blocked and not self.path_left_blocked:
            t.angular.z = max(t.angular.z, 0.45)
            v *= 0.65
        elif self.path_left_blocked and self.path_right_blocked:
            v *= 0.40

        if self.front_min < OBSTACLE_DIST:
            pref = turn_pref if turn_pref is not None else self._pick_turn_dir()
            linear, ang_mult = self._resolve_turn(pref)
            v = linear
            t.angular.z = ang_mult * W_TURN_OBSTACLE

        # Sem isso o motor segue empurrando contra a roda enganchada -> tombo.
        if self.wheel_left_close or self.wheel_right_close:
            pref = 1 if self.wheel_left_close else -1
            linear, ang_mult = self._resolve_turn(pref)
            v = linear
            t.angular.z = ang_mult * W_TURN_OBSTACLE

        t.linear.x = v
        return t

    def _navigate_step(self) -> Twist:
        """Convergencia para a bandeira via cx do /flag_detection."""
        cx = self.flag[1] if self.flag[0] else self.last_flag_cx * 0.4
        return self._navigate_toward(cx)

    def _return_step(self):
        """Volta a pose inicial reusando a MESMA navegacao/desvio que leva ate a
        bandeira (_navigate_toward) -- proven. So troca o alvo: em vez do cx da
        visao, um cx derivado do rumo para a base. Retorna (twist, chegou)."""
        if self._cur_pose is None or self._home_pose is None:
            return Twist(), False
        x, y, yaw = self._cur_pose
        dx = self._home_pose[0] - x
        dy = self._home_pose[1] - y
        if math.hypot(dx, dy) < GOAL_RADIUS:
            return Twist(), True

        err = math.atan2(dy, dx) - yaw
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi

        # cx equivalente ao da visao: base a esquerda (err>0) -> cx<0, como uma
        # bandeira a esquerda. |cx|=1 (base atras) -> gira no lugar e desvia
        # exatamente como na navegacao para a bandeira.
        cx_home = max(-1.0, min(1.0, -err / RETURN_TURN_IN_PLACE))
        # ao desviar de obstaculo, vira para o lado da base (err>0 = base a
        # esquerda = +1), nunca pelo lado "errado" da serpentina.
        turn_pref = 1 if err >= 0 else -1
        return self._navigate_toward(cx_home, turn_pref), False

    def _position_step(self):
        """Aproximacao final em duas fases para nao derrubar o mastro:
        (1) alinha girando no lugar ate o mastro ficar centrado; (2) so entao
        avanca reto e devagar. As pincas abertas vao ~0.5m a frente (alem do
        mastro), entao girar enquanto avanca varre o mastro de lado e o derruba."""
        t = Twist()
        if self._touch_latched:
            return t, True
        cx = self.flag[1]
        d = self.front_min
        err = d - self.capture_distance
        flag_close = self.flag[0] and self.flag[3] >= CAPTURE_MIN_AREA

        # Fase 1: desalinhado -> gira no lugar (sem avancar).
        if abs(cx) > CAPTURE_CX_ALIGN:
            t.angular.z = -1.2 * cx
            return t, False

        # Fase 2: alinhado. Mastro na zona de pega -> latch.
        if flag_close and abs(cx) < CAPTURE_CX_LATCH and d < self.capture_distance + 0.05:
            self._touch_latched = True
            return t, True

        # Creep reto e lento; correcao de heading bem suave para nao varrer.
        t.angular.z = -0.4 * cx
        t.linear.x = max(0.03, min(0.10, 0.5 * err))
        done = flag_close and abs(err) < CAPTURE_ERR_DONE and abs(cx) < CAPTURE_CX_DONE
        return t, done

    def _gripper(self, elevation: float, right: float, left: float):
        """Publica comando da garra: [elevacao, abertura_dir, abertura_esq]."""
        msg = Float64MultiArray()
        msg.data = [elevation, right, left]
        self.gripper_pub.publish(msg)

    def _set_state(self, new_state: State):
        """Transicao de estado com log e republicacao em /mission_state."""
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

    def _is_active_state(self) -> bool:
        """Estados em que o robo deveria estar progredindo (watchdog se aplica)."""
        return self.state in (
            State.EXPLORANDO, State.NAVEGANDO_PARA_BANDEIRA,
            State.REDETECTANDO_BANDEIRA, State.POSICIONANDO_PARA_COLETA,
            State.RETORNANDO_BASE,
        )

    def _watchdog_step(self, now: float):
        """Detecta travamento via /odom_gt. Retorna twist de recovery se preciso,
        ou None pra deixar a FSM rodar normal."""
        if now < self._recovery_until:
            elapsed = RECOVERY_TOTAL - (self._recovery_until - now)
            t = Twist()
            if elapsed < RECOVERY_BACKUP_DUR and self.rear_min > REAR_SAFE_DIST:
                t.linear.x = RECOVERY_BACKUP_SPEED
            else:
                t.angular.z = RECOVERY_TURN_SPEED * self._recovery_dir
            return t

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

        # REDETECTANDO eh giro puro: aceita yaw liquido como progresso. O
        # RETORNANDO NAO entra aqui de proposito: se ficar oscilando sem
        # avancar (encurralado), o watchdog precisa disparar recovery p/ escapar.
        if not progressed and self.state == State.REDETECTANDO_BANDEIRA:
            dyaw = self._cur_pose[2] - self._last_progress_pose[2]
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            if abs(dyaw) > STUCK_YAW_NET:
                progressed = True

        if progressed:
            self._last_progress_pose = self._cur_pose
            self._last_progress_t = now
            return None

        if (now - self._last_progress_t) > STUCK_TIME:
            self._recovery_until = now + RECOVERY_TOTAL
            self._recovery_dir = -self._recovery_dir
            self._last_progress_pose = self._cur_pose
            self._last_progress_t = now
            self.get_logger().warn(
                f'[watchdog] sem progresso em {self.state.value}, recovery dir={self._recovery_dir}')
            t = Twist()
            t.linear.x = RECOVERY_BACKUP_SPEED
            return t

        return None

    def _filter_pendulation(self, lin: float, ang: float, now: float) -> float:
        """Histerese de sinal em angular.z para evitar oscilacao L/R quando
        o robo esta parado. Em movimento linear o filtro nao atua."""
        new_sign = 0
        if abs(ang) > 0.05:
            new_sign = 1 if ang > 0 else -1

        if abs(lin) > 0.05:
            self._turn_sign = new_sign
            if new_sign != 0:
                self._turn_sign_t = now
            return ang

        if new_sign == 0 or new_sign == self._turn_sign:
            self._turn_sign = new_sign
            if new_sign != 0:
                self._turn_sign_t = now
            return ang

        # Flip dentro do commit: forca direcao antiga para nao pendular.
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
