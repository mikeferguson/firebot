#!/usr/bin/env python3

# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import RegionOfInterest
from math import acos, pi, radians
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
        # Subscribe to the lepton driver flame detection
        self.roi = 0
        self.roi_sub = self.create_subscription(RegionOfInterest, 'lepton/flame_region', self.roi_callback, 1)

        # Subscribe to odometry for local panning motions
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0
        self.odom_sub = self.create_subscription(Odometry, 'base_controller/odom', self.odom_callback, 1)

        # Publish twist command for local panning motions
        self.cmd_pub = self.create_publisher(Twist, 'base_controller/command', 1)
        self.search_targets = None

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
            # Rotate the robot looking for candle
            if self.search_targets == None:
                self.search_targets = [self.odom_th - pi / 2, self.odom_th + pi / 2]
                self.roi = None
            if self.pan_to_targets(self.search_targets):
                # Have panned in both directions, still no candle
                self.state = STATE_NAV_TO_NEXT_ROOM
            elif self.roi is not None:
                # We have a candle! Stop the robot
                self.stop_motion()
                self.state = STATE_APPROACH_FIRE

        elif self.state == STATE_APPROACH_FIRE:
            # TODO: approach the fire
            pass

        elif self.state == STATE_EXTINGUISH_FIRE:
            # TODO: turn on fan, rotate back and forth
            pass

    #
    # Fire Search Routines
    #

    ## @brief Pan towards the first of possibly several targets
    ##
    ## When a target is reached (based on odometry), it will be
    ## removed from the list of targets.
    def pan_to_targets(self, targets):
        if abs(self.targets[0] - self.odom_th) < radians(5):
            # Goal is met, pop it off
            targets = targets[1: -1]
            if len(targets) == 0:
                # Note that we have panned to all targets
                return True
        # Compute direction to turn, and send command
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.targets[0] - self.odom_th
        msg.angular.z = min(msg.angular.z, 0.25)
        msg.angular.z = max(msg.angular.z, -0.25)
        self.cmd_pub.publish(msg)
        # Not done panning yet
        return False

    ## @brief Stop all motion
    def stop_motion(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    #
    # ROS2 Callbacks
    #

    ## @brief Callback for ROI of detected flame (if any)
    def roi_callback(self, msg):
        self.roi = msg

    ## @brief Odometry from robot base
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.x
        self.odom_th = acos(msg.pose.pose.orientation.w) * 2.0


if __name__=="__main__":
    rclpy.init()
    node = FirebotStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
