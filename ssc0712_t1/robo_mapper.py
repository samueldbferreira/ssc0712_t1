#!/usr/bin/env python3
"""Publica TF estatico map->odom_gt e um OccupancyGrid simples marcando
as celulas visitadas pelo robo. Usado pelo RViz."""
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import OccupancyGrid
from scipy.spatial.transform import Rotation as R
from tf2_ros import StaticTransformBroadcaster


class RoboMapper(Node):
    def __init__(self):
        super().__init__('robo_mapper')

        self.create_subscription(Pose, '/model/prm_robot/pose', self.odom_callback, 10)
        self.timer = self.create_timer(0.5, self.atualiza_mapa)

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        self.grid_size = 50
        self.resolution = 0.25
        self.grid_map = -np.ones((self.grid_size, self.grid_size), dtype=np.int8)

        self.map_pub = self.create_publisher(OccupancyGrid, '/grid_map', 10)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = "map"
        static_tf.child_frame_id = "odom_gt"
        static_tf.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(static_tf)

    def odom_callback(self, msg: Pose):
        self.x = msg.position.x
        self.y = msg.position.y
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.heading = R.from_quat(q).as_euler('xyz', degrees=False)[2]

    def world_to_grid(self, x, y):
        origin_offset = self.grid_size * self.resolution / 2
        gx = int((x + origin_offset) / self.resolution)
        gy = int((y + origin_offset) / self.resolution)
        return gx, gy

    def atualiza_mapa(self):
        gx, gy = self.world_to_grid(self.x, self.y)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.grid_map[gy, gx] = 100
        self.publish_occupancy_grid()

    def publish_occupancy_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size

        origin = Pose()
        origin.position.x = -(self.grid_size * self.resolution) / 2
        origin.position.y = -(self.grid_size * self.resolution) / 2
        origin.orientation.w = 1.0
        grid_msg.info.origin = origin

        grid_msg.data = self.grid_map.flatten().tolist()
        self.map_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoboMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
