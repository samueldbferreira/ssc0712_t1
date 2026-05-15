#!/usr/bin/env python3
"""Detector visual da bandeira.

Le /robot_cam/labels_map (imagem mono onde cada pixel = label id do plugin
de segmentacao semantica do Gazebo). Publica em /flag_detection um
Float32MultiArray com layout:

    data[0] = detected      (0.0 ou 1.0)
    data[1] = center_x_norm ([-1, 1], 0 = centro horizontal da imagem)
    data[2] = center_y_norm ([-1, 1], 0 = centro vertical)
    data[3] = area_ratio    (fracao da imagem ocupada pela bandeira, 0..1)

A maquina de estados consome esses 4 valores para decidir transicoes.

Parametro ROS:
    flag_label (int, default 25): id do label a procurar. Em mapas CTF
        (arena_cilindros, arena_paredes, empty_arena) a bandeira do time
        adversario (azul) tem label 25. No arena.sdf legado, a bandeira
        tem label 40. Pode-se passar uma lista de labels separados por
        virgula (ex: "25,40") para suportar varios mapas com a mesma
        configuracao do detector.
"""
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from cv_bridge import CvBridge
import numpy as np

MIN_PIXELS = 30  # ignora ruido isolado de poucos pixels


class FlagDetector(Node):
    def __init__(self):
        super().__init__('flag_detector')
        self.bridge = CvBridge()

        # Parametro: aceita int unico, string com 1 label ("40"), ou string
        # com varios labels separados por virgula ("25,40").
        # dynamic_typing=True permite que o launch passe int sem conflitar
        # com o default string (Humble rejeita override de tipo diferente
        # silenciosamente sem isso, deixando o detector com [25] default).
        self.declare_parameter(
            'flag_label', '25',
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )
        val = self.get_parameter('flag_label').value
        if isinstance(val, int):
            self.flag_labels = [val]
        else:
            raw = str(val)
            try:
                self.flag_labels = [int(x.strip()) for x in raw.split(',') if x.strip()]
            except ValueError:
                self.get_logger().warn(f"flag_label invalido: '{raw}', usando [25]")
                self.flag_labels = [25]
        self.get_logger().info(f'procurando bandeira(s) label={self.flag_labels}')

        self.sub = self.create_subscription(
            Image, '/robot_cam/labels_map', self.on_image, 10
        )
        self.pub = self.create_publisher(Float32MultiArray, '/flag_detection', 10)

        # Para debug periodico no log
        self._last_log = self.get_clock().now()

    def on_image(self, msg: Image):
        # labels_map vem com encoding "mono8" (cada pixel = label id).
        # Em algumas versoes pode vir como rgb8 (R=label, G=instancia, B=instancia).
        # Tratamos ambos os casos.
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge falhou: {e}')
            return

        if img.ndim == 3:
            # canal R guarda o label (formato segmentacao do ignition gazebo)
            label_plane = img[:, :, 0]
        else:
            label_plane = img

        # Mascara: True para qualquer label da lista
        mask = np.isin(label_plane, self.flag_labels)
        n_pixels = int(mask.sum())
        h, w = label_plane.shape[:2]
        total = h * w

        out = Float32MultiArray()
        out.layout = MultiArrayLayout(dim=[MultiArrayDimension(
            label='flag', size=4, stride=4)])

        if n_pixels >= MIN_PIXELS:
            ys, xs = np.where(mask)
            cx = float(xs.mean())
            cy = float(ys.mean())
            center_x_norm = (cx - w / 2.0) / (w / 2.0)
            center_y_norm = (cy - h / 2.0) / (h / 2.0)
            area_ratio = n_pixels / float(total)
            out.data = [1.0, center_x_norm, center_y_norm, area_ratio]
        else:
            out.data = [0.0, 0.0, 0.0, 0.0]

        self.pub.publish(out)

        # Log a cada 2s
        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 2e9:
            self._last_log = now
            # DIAG: lista TODOS os labels unicos presentes na imagem.
            # Se label 40 (ou o que self.flag_labels esperar) nao aparece
            # aqui, o problema esta no Label plugin do SDF, nao na deteccao.
            unique_labels, counts = np.unique(label_plane, return_counts=True)
            top = sorted(zip(unique_labels.tolist(), counts.tolist()),
                         key=lambda kv: -kv[1])[:8]
            self.get_logger().info(
                f'DIAG labels presentes (top): {top} | '
                f'procurando={self.flag_labels} | '
                f'encoding={msg.encoding} shape={label_plane.shape}'
            )
            if out.data[0] > 0.5:
                self.get_logger().info(
                    f'bandeira: cx={out.data[1]:+.2f} cy={out.data[2]:+.2f} '
                    f'area={out.data[3]*100:.2f}%'
                )
            else:
                self.get_logger().info('bandeira: nao visivel')


def main(args=None):
    rclpy.init(args=args)
    node = FlagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
