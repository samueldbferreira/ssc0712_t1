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
    capture_distance (float, default 0.55): alvo de distancia frontal final.
    close_area_ratio (float, default 0.04): area ratio que dispara POSICIONANDO.
    lost_frames_to_redetect (int, default 12): N frames sem bandeira -> REDETECTANDO.
"""
import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
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
V_POSITION = 0.08
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

# Roda: borda externa a 0.20m do centro. Abaixo desse limiar o movimento
# angular pode aproximar a roda o suficiente para enganchar num obstaculo.
WHEEL_SAFE_DIST = 0.40


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 5.0)
        # Distancia frontal alvo na parada final (LIDAR). 0.65m da ~0.25m
        # de folga ao braco (que estica 0.4m a frente).
        self.declare_parameter('capture_distance', 0.65)
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

        # Estado sensorial
        self.flag = (False, 0.0, 0.0, 0.0)
        self.lost_counter = 0
        self.last_flag_cx = 0.0   # ultimo cx valido quando bandeira estava visivel
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')
        # Zonas laterais puras (90°±40°) para detectar enganche de roda.
        self.side_left_min = float('inf')
        self.side_right_min = float('inf')
        # Corridor check — dois limiares:
        #   path_*_blocked  (zona amarela 0.35m): vira e reduz velocidade
        #   inner_*_blocked (zona vermelha 0.24m): para completamente e vira forte
        self.path_left_blocked = False
        self.path_right_blocked = False
        self.inner_left_blocked = False
        self.inner_right_blocked = False
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
        self.left_min = window_min(60, 30)
        self.right_min = window_min(300, 30)
        # Janela alargada: roda esta em x=-0.12m do centro; quando a roda esta
        # ao lado do obstaculo, o LIDAR ve esse obstaculo a ~121° (nao a 90°).
        # 90°±40° = 50°–130° cobre toda a zona de enganche da roda.
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

    def on_start(self, msg: Bool):
        if msg.data and self.state == State.AGUARDANDO_COMANDO:
            self._set_state(State.EXPLORANDO)

    # ------------------------------------------------------------------
    def step(self):
        if (self.state == State.AGUARDANDO_COMANDO and self.auto_start
                and (time.monotonic() - self.start_time) > self.auto_start_delay):
            self._set_state(State.EXPLORANDO)

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
            if (time.monotonic() - self.state_entered_at) > 0.5:
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
            twist.angular.z = W_TURN_SEARCH * (-1.0 if self.last_flag_cx > 0 else 1.0)
            if self.flag[0]:
                self._set_state(State.NAVEGANDO_PARA_BANDEIRA)
            elif (time.monotonic() - self.state_entered_at) > 8.0:
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
            t = time.monotonic() - self.state_entered_at
            self._celebrate(t)

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    def _pick_turn_dir(self) -> int:
        """Retorna +1 (esquerda) ou -1 (direita) baseado na fase atual da
        serpentina.

        Antes escolhia o lado com mais espaco aberto (left_min vs right_min),
        mas isso causava U-turn na fronteira entre area com obstaculos e area
        vazia — a vazia parecia 'mais aberta', entao o desvio mandava o robo
        de volta pelo caminho ja percorrido. Agora segue o sentido natural
        do sweep da serpentina: nao tem preferencia por espaco vazio.
        """
        return 1 if math.sin(self.serpentine_phase * 0.7) >= 0 else -1

    def _explore_step(self) -> Twist:
        """Anda para frente com serpentina adaptativa. Se LIDAR ve obstaculo
        frontal, gira para o lado mais aberto. Corridor checks fazem desvios
        sutis para obstaculos laterais entrando no caminho."""
        t = Twist()

        if self.front_min < OBSTACLE_DIST:
            t.angular.z = W_TURN_OBSTACLE * self._pick_turn_dir()
            t.linear.x = 0.0
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
            else:
                self.serpentine_phase += 0.1
                # Velocidade e amplitude escalam com folga frontal:
                #   espaco aberto (front>3m): rapido + serpentina larga,
                #     cobre area lateral ampla, FOV de 90 graus pega
                #     bandeiras fora do eixo
                #   espaco medio (1.5-3m): velocidade e amplitude normais
                #   espaco apertado (<1.5m): mais devagar, serpentina
                #     minima — deixa corridor checks dominarem
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
        return t

    def _navigate_step(self) -> Twist:
        """Vai em direcao a bandeira usando cx_norm como erro de heading.
        Se ha obstaculo, contorna pelo lado mais aberto. Durante o dodge
        zera v: o braco (0.4m a frente, invisivel ao LIDAR) pode tocar o
        obstaculo se avancar enquanto rotaciona."""
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
            v = 0.0
            t.angular.z = W_TURN_OBSTACLE * self._pick_turn_dir()
        t.linear.x = v
        return t

    def _position_step(self):
        """Posicionamento fino: para a `capture_distance` da bandeira, centrada."""
        t = Twist()
        cx = self.flag[1]
        t.angular.z = -1.5 * cx
        d = self.front_min
        err = d - self.capture_distance
        v = max(-0.05, min(0.12, 0.5 * err))
        if abs(cx) > 0.15:
            v = 0.0
        t.linear.x = v
        done = abs(err) < 0.05 and abs(cx) < 0.07
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
