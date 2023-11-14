#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

from poses import HOME


class Localization:

    def __init__(self, node):
        self.pub = node.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.node = node

    def localize_at_home(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose = HOME
        msg.pose.covariance[0] = 0.0125
        msg.pose.covariance[7] = 0.0125
        msg.pose.covariance[35] = 0.06853891909122467
        self.pub.publish(msg)


if __name__=="__main__":
    rclpy.init()
    node = Node("localize")

    localize = Localization(node)
    localize.localize_at_home()

    rclpy.spin(node)
