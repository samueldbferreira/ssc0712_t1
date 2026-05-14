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

from sensor_msgs.msg import LaserScan, Imu, Image
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


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')

        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_start_delay', 5.0)
        # Distancia frontal alvo na parada final (LIDAR). 0.65m da ~0.25m
        # de folga ao braco (que estica 0.4m a frente).
        self.declare_parameter('capture_distance', 0.65)
        self.declare_parameter('close_area_ratio', 0.04)
        self.declare_parameter('lost_frames_to_redetect', 12)

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
        self.front_min = float('inf')
        self.left_min = float('inf')
        self.right_min = float('inf')

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
            twist.angular.z = W_TURN_SEARCH
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
    def _explore_step(self) -> Twist:
        """Anda para frente com leve serpentina. Se LIDAR ve obstaculo
        frontal, gira para o lado mais aberto."""
        t = Twist()
        if self.front_min < OBSTACLE_DIST:
            if self.left_min > self.right_min:
                t.angular.z = W_TURN_OBSTACLE
            else:
                t.angular.z = -W_TURN_OBSTACLE
            t.linear.x = 0.0
        else:
            self.serpentine_phase += 0.1
            t.linear.x = V_EXPLORE
            t.angular.z = 0.3 * math.sin(self.serpentine_phase * 0.7)
        return t

    def _navigate_step(self) -> Twist:
        """Vai em direcao a bandeira usando cx_norm como erro de heading.
        Se ha obstaculo, contorna pelo lado mais aberto. Durante o dodge
        zera v: o braco (0.4m a frente, invisivel ao LIDAR) pode tocar o
        obstaculo se avancar enquanto rotaciona."""
        t = Twist()
        cx = self.flag[1]
        t.angular.z = -1.2 * cx
        v = V_NAV * max(0.0, 1.0 - abs(cx))
        if self.front_min < OBSTACLE_DIST:
            v = 0.0
            side_bias = W_TURN_OBSTACLE
            if self.right_min > self.left_min:
                side_bias = -W_TURN_OBSTACLE
            t.angular.z = side_bias
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
