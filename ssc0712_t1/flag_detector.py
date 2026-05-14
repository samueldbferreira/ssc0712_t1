#!/usr/bin/env python3
"""Detector visual da bandeira.

Le /robot_cam/labels_map (imagem mono onde cada pixel = label id do plugin
de segmentacao semantica do Gazebo). A bandeira tem label 40 (definido em
arena.sdf). Publica em /flag_detection um Float32MultiArray com layout:

    data[0] = detected      (0.0 ou 1.0)
    data[1] = center_x_norm ([-1, 1], 0 = centro horizontal da imagem)
    data[2] = center_y_norm ([-1, 1], 0 = centro vertical)
    data[3] = area_ratio    (fracao da imagem ocupada pela bandeira, 0..1)

A maquina de estados consome esses 4 valores para decidir transicoes.
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from cv_bridge import CvBridge
import numpy as np

FLAG_LABEL = 40
MIN_PIXELS = 30  # ignora ruido isolado de poucos pixels


class FlagDetector(Node):
    def __init__(self):
        super().__init__('flag_detector')
        self.bridge = CvBridge()

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

        mask = (label_plane == FLAG_LABEL)
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
