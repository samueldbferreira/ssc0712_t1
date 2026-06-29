#!/usr/bin/env python3
"""Detector visual da bandeira via plugin de labels do Gazebo.

Le /robot_cam/labels_map (cada pixel = label id) e publica em
/flag_detection um Float32MultiArray [detected, cx_norm, cy_norm, area_ratio].
Parametro flag_label aceita int ou string com lista separada por virgula.
"""
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

from cv_bridge import CvBridge
import numpy as np

MIN_PIXELS = 10   # ~6m de alcance considerando so o mastro (0.06m de largura)


class FlagDetector(Node):
    def __init__(self):
        super().__init__('flag_detector')
        self.bridge = CvBridge()

        # dynamic_typing permite o launch sobrescrever um string default com int.
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

        self._last_log = self.get_clock().now()

    def on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge falhou: {e}')
            return

        # Algumas versoes do plugin enviam rgb8 com label no canal R.
        label_plane = img[:, :, 0] if img.ndim == 3 else img

        mask = np.isin(label_plane, self.flag_labels)
        n_pixels = int(mask.sum())
        h, w = label_plane.shape[:2]
        total = h * w

        out = Float32MultiArray()
        out.layout = MultiArrayLayout(dim=[MultiArrayDimension(
            label='flag', size=4, stride=4)])

        if n_pixels >= MIN_PIXELS:
            ys, xs = np.where(mask)
            # O painel fica no alto e deslocado lateralmente: o centroide do
            # conjunto (mastro+painel) nao coincide com o mastro. A garra agarra
            # o mastro, entao o cx de alinhamento usa so o terco inferior da
            # mascara, onde so existe o mastro (linhas de baixo da imagem).
            y_min, y_max = int(ys.min()), int(ys.max())
            cutoff = y_max - 0.35 * (y_max - y_min)
            pole_xs = xs[ys >= cutoff]
            cx = float(pole_xs.mean()) if pole_xs.size else float(xs.mean())
            cy = float(ys.mean())
            out.data = [
                1.0,
                (cx - w / 2.0) / (w / 2.0),
                (cy - h / 2.0) / (h / 2.0),
                n_pixels / float(total),
            ]
        else:
            out.data = [0.0, 0.0, 0.0, 0.0]

        self.pub.publish(out)

        now = self.get_clock().now()
        if (now - self._last_log).nanoseconds > 2e9:
            self._last_log = now
            unique_labels, counts = np.unique(label_plane, return_counts=True)
            top = sorted(zip(unique_labels.tolist(), counts.tolist()),
                         key=lambda kv: -kv[1])[:8]
            self.get_logger().info(
                f'labels presentes (top): {top} | '
                f'procurando={self.flag_labels} | '
                f'encoding={msg.encoding} shape={label_plane.shape}'
            )
            if out.data[0] > 0.5:
                self.get_logger().info(
                    f'bandeira: cx={out.data[1]:+.2f} cy={out.data[2]:+.2f} '
                    f'area={out.data[3]*100:.2f}%'
                )


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
