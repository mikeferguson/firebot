#!/usr/bin/env python3

# ROS2
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Subroutines
from poses import HOME, ROOMS

# States
STATE_WAIT_FOR_START = 0
STATE_NAV_TO_NEXT_ROOM = 1
STATE_NAV_IN_PROGRESS = 2
STATE_SEARCH_ROOM = 3
STATE_APPROACH_FIRE = 4
STATE_EXTINGUISH_FIRE = 5
STATE_RETURN_HOME = 6


def getStamped(pose):
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose = poses
    return ps


class FirebotStateMachine(Node):

    next_room = -1
    state = STATE_WAIT_FOR_START

    def __init__(self):
        # Action interface to nav2
        self.nav2 = BasicNavigator()
        self.get_logger().info("Waiting for nav2")
        self.nav2.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active")

        self.timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        if self.state == STATE_WAIT_FOR_START:
            # TODO: wait for start button to be pressed
            #       once pressed, localize robot to start pose and set state to nav
            self.get_logger().info("Start button pressed")
            self.nav2.setInitialPose(getStamped(HOME))
            self.state = STATE_NAV_TO_NEXT_ROOM

        elif self.state == STATE_NAV_TO_NEXT_ROOM:
            self.next_room += 1
            self.get_logger().info("nav2 to room %d", self.next_room)
            self.nav2.goToPose(getStamped(ROOMS[self.next_room]))
            self.state = STATE_NAV_IN_PROGRESS

        elif self.state == STATE_NAV_IN_PROGRESS:
            if self.nav2.isTaskComplete():
                result = self.nav2.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.state = STATE_SEARCH_ROOM
                else:
                    # Send goal again
                    self.get_logger().info("Retry nav2 to room %d", self.next_room)
                    self.nav2.goToPose(getStamped(ROOMS[self.next_room]))

        elif self.state == STATE_SEARCH_ROOM:
            # TODO: search for the candle
            self.state = STATE_NAV_TO_NEXT_ROOM

        elif self.state == STATE_APPROACH_FIRE:
            # TODO: approach the fire
            pass

        elif self.state == STATE_EXTINGUISH_FIRE:
            # TODO: turn on fan, rotate back and forth
            pass


if __name__=="__main__":
    rclpy.init()
    node = FirebotStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
