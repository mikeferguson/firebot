#!/usr/bin/env python3

from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

use_ndt_map = True

if use_ndt_map:

    HOME = Pose()
    HOME.position.x = 0.0
    HOME.position.y = 0.0
    HOME.orientation.z = 0.0343038
    HOME.orientation.w = 0.999411

    ROOM1 = Pose()
    ROOM1.position.x = 0.499386
    ROOM1.position.y = -0.587836
    ROOM1.orientation.z = 0.737776
    ROOM1.orientation.w = 0.675045

    ROOM2 = Pose()
    ROOM2.position.x = 0.483585
    ROOM2.position.y = -1.53935
    ROOM2.orientation.z = 0.04777233
    ROOM2.orientation.w = 0.998861

    ROOM3 = Pose()
    ROOM3.position.x = -0.347449
    ROOM3.position.y = -1.9449
    ROOM3.orientation.z = -0.999439
    ROOM3.orientation.w = 0.0335024

    ROOM4 = Pose()
    ROOM4.position.x = -0.768552
    ROOM4.position.y = -0.564789
    ROOM4.orientation.z = 0.730972
    ROOM4.orientation.w = 0.682407

else:
    HOME = Pose()
    HOME.position.x = 1.2030736207962036
    HOME.position.y = 2.258392810821533
    HOME.orientation.w = 1.0

    ROOM1 = Pose()
    ROOM1.position.x = 1.616312861442566
    ROOM1.position.y = 1.6851954460144043
    ROOM1.orientation.z = 0.7072022009881881
    ROOM1.orientation.w = 0.7070113485068414

    ROOM2 = Pose()
    ROOM2.position.x = 0.5802170038223267
    ROOM2.position.y = 1.7951760292053223
    ROOM2.orientation.z = 0.7072022009881881
    ROOM2.orientation.w = 0.7070113485068414

    ROOM3 = Pose()
    ROOM3.position.x = 0.8934671878814697
    ROOM3.position.y = 0.5834417343139648
    ROOM3.orientation.z = 0.9999994215024537
    ROOM3.orientation.w = 0.0010756369080212524

    ROOM4 = Pose()
    ROOM4.position.x = 1.4859161376953125
    ROOM4.position.y = 0.9865974187850952
    ROOM4.orientation.z = 0.0
    ROOM4.orientation.w = 1.0

ROOMS = [ROOM1, ROOM2, ROOM3, ROOM4]

if __name__ == '__main__':
    rclpy.init()
    node = Node('localize')

    pub = node.create_publisher(MarkerArray, 'rooms', 1)

    msg = MarkerArray()
    marker = Marker()
    marker_id = 1
    for room in ROOMS:
        marker = Marker()
        marker.ns = 'ROOM'
        marker.id = marker_id
        marker.header.frame_id = 'map'
        marker.pose = room
        marker.type = Marker.ARROW
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0
        msg.markers.append(marker)
        marker_id += 1

    marker = Marker()
    marker.ns = 'HOME'
    marker.header.frame_id = 'map'
    marker.pose = HOME
    marker.type = Marker.ARROW
    marker.scale.x = 1.0
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.b = 1.0
    marker.color.a = 1.0
    msg.markers.append(marker)

    pub.publish(msg)
    rclpy.spin(node)
