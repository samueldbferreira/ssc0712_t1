#!/usr/bin/env python3
"""Republica /model/prm_robot/pose como Odometry em /odom_gt e TF odom_gt->base_link."""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class GroundTruthOdomPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_odom_publisher')

        self.create_subscription(Pose, '/model/prm_robot/pose', self.pose_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom_gt', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.base_frame = 'base_link'
        self.odom_frame = 'odom_gt'

    def pose_callback(self, msg: Pose):
        now = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = msg
        self.odom_pub.publish(odom_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.position.x
        tf_msg.transform.translation.y = msg.position.y
        tf_msg.transform.translation.z = msg.position.z
        tf_msg.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
