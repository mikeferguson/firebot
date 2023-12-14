#!/usr/bin/env python3

# Standard library
from math import atan2, pi, radians
import time

# ROS2
from action_msgs.msg import GoalStatus
from angles import shortest_angular_distance
from etherbotix import Etherbotix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from poses import HOME, ROOMS
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
import tf2_geometry_msgs
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# States
STATE_CONFIGURE = 0
STATE_WAIT_FOR_PUSH = 1
STATE_WAIT_FOR_RELEASE = 2
STATE_NAV_TO_NEXT_ROOM = 3
STATE_NAV_IN_PROGRESS = 4
STATE_SEARCH_ROOM = 5
STATE_APPROACH_FIRE = 6
STATE_EXTINGUISH_FIRE = 7
STATE_RETURN_HOME = 8
STATE_DONE = 9

# Etherbotix I/O
START_BUTTON = 0x80
START_BUTTON_PIN = 7
FAN_PIN = 6


class FirebotStateMachine(Node):

    next_room = -1
    state = STATE_CONFIGURE

    def __init__(self):
        super().__init__('firebot')

        # Subscribe to the lepton driver flame detection
        self.roi = 0
        self.roi_sub = self.create_subscription(RegionOfInterest, 'lepton/flame_region',
                                                self.roi_callback, 1)

        # Subscribe to odometry for local panning motions
        self.odom_x = None
        self.odom_y = None
        self.odom_th = None
        self.odom_sub = self.create_subscription(Odometry, 'base_controller/odom',
                                                 self.odom_callback, 1)

        # Create an Etherbotix instance for interacting with IO
        self.etherbotix = Etherbotix('192.168.0.42', 6707)

        # TF interface
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Publish twist command for local panning motions
        self.cmd_pub = self.create_publisher(Twist, 'base_controller/command', 1)
        self.search_targets = None

        # Localization
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)

        # Action interface to nav2
        self.nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        if self.odom_th is None:
            self.get_logger().warn('Not yet ready - waiting for odom')
            return

        if self.state == STATE_CONFIGURE:
            # Set start button as input, with pullup
            if not self.etherbotix.set_digital_pin(START_BUTTON_PIN, 1, 0):
                self.get_logger().warn('Etherbotix not yet responding')
                self.etherbotix.update()
                return

            # Extra update so that we have the correct digital config
            self.etherbotix.update()
            time.sleep(0.5)

            # Etherbotix is now configured - wait for start button
            self.state = STATE_WAIT_FOR_PUSH

        if self.state == STATE_WAIT_FOR_PUSH:
            # Button must be pressed
            self.etherbotix.update()
            if self.etherbotix.get_digital_in() & START_BUTTON > 0:
                return

            self.get_logger().info('Start button pressed')
            self.state = STATE_WAIT_FOR_RELEASE

        elif self.state == STATE_WAIT_FOR_RELEASE:
            # Button must be released
            self.etherbotix.update()
            if self.etherbotix.get_digital_in() & START_BUTTON == 0:
                return

            self.get_logger().info('Start button released - go!')
            self.localize(HOME)
            self.state = STATE_NAV_TO_NEXT_ROOM

        elif self.state == STATE_NAV_TO_NEXT_ROOM:
            self.next_room += 1
            self.get_logger().info('Navigating to room %d' % self.next_room)
            self.go_to_pose(ROOMS[self.next_room])
            self.state = STATE_NAV_IN_PROGRESS

        elif self.state == STATE_NAV_IN_PROGRESS:
            if self.nav2_result is not None:
                if self.nav2_result == GoalStatus.STATUS_SUCCEEDED:
                    self.state = STATE_SEARCH_ROOM
                else:
                    # Send goal again
                    self.get_logger().info('Retry nav2 to room %d' % self.next_room)
                    self.go_to_pose(ROOMS[self.next_room])

        elif self.state == STATE_SEARCH_ROOM:
            # Rotate the robot looking for candle
            if self.search_targets is None:
                self.get_logger().info('Starting to search room')
                self.search_targets = [self.odom_th - pi / 2, self.odom_th + pi / 2]
                self.roi = None
            if self.pan_to_targets():
                # Have panned in both directions, still no candle
                self.search_targets = None
                self.state = STATE_NAV_TO_NEXT_ROOM
            elif self.roi is not None:
                # We have a candle! Stop the robot
                self.get_logger().info('Candle found!')
                self.stop_motion()
                self.state = STATE_APPROACH_FIRE

        elif self.state == STATE_APPROACH_FIRE:
            # Approach the fire
            msg = Twist()
            # TODO: better stopping condition?
            if self.roi.width > 12:
                # Stop the approach and put out the fire
                self.get_logger().info('Approach complete - attempt to extinguish!')
                self.cmd_pub.publish(msg)
                self.state = STATE_EXTINGUISH_FIRE
                # Setup panning action
                self.search_targets = [self.odom_th + pi / 4, self.odom_th - pi / 8, self.odom_th]
                return
            msg.linear.x = 0.035
            msg.angular.z = 0.02 * (self.roi.y_offset - 80)
            msg.angular.z = min(msg.angular.z, 0.35)
            msg.angular.z = max(msg.angular.z, -0.35)
            self.cmd_pub.publish(msg)

        elif self.state == STATE_EXTINGUISH_FIRE:
            # Turn on fan
            self.enable_fan(True)
            # Rotate back and forth, up to 0.5 rad/s
            if self.pan_to_targets(0.5):
                if self.roi is None:
                    self.get_logger().warn('Fire is extinguished!')
                    self.state = STATE_RETURN_HOME
                else:
                    # Still seeing fire - go again
                    self.get_logger().info('Attempting to extinguish (again)')
                    self.roi = None
                    self.search_targets = [self.odom_th + pi / 4,
                                           self.odom_th - pi / 8, self.odom_th]

        elif self.state == STATE_RETURN_HOME:
            self.enable_fan(False)
            # TODO: actually return home
            self.state = STATE_DONE

        elif self.state == STATE_DONE:
            self.get_logger().info('Done')
            self.timer.cancel()

    #
    # Fire Search Routines
    #

    # @brief Pan towards the first of possibly several targets
    #
    # When a target is reached (based on odometry), it will be
    # removed from the list of targets.
    def pan_to_targets(self, velocity_lim=0.35):
        # Note if we have panned to all targets
        if self.search_targets is None or len(self.search_targets) == 0:
            self.stop_motion()
            return True

        # Select next target
        target = self.search_targets[0]

        # Have we reached the next target?
        if abs(shortest_angular_distance(target, self.odom_th)) < radians(5):
            # Goal is met, pop it off
            self.search_targets = self.search_targets[1:]
            # Note if we have panned to all targets
            if len(self.search_targets) == 0:
                self.stop_motion()
                return True
            target = self.search_targets[0]

        # Compute direction to turn, and send command
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 2 * shortest_angular_distance(self.odom_th, target)
        msg.angular.z = min(msg.angular.z, velocity_lim)
        msg.angular.z = max(msg.angular.z, -velocity_lim)
        self.cmd_pub.publish(msg)

        # Not done panning yet
        return False

    # @brief Stop all motion
    def stop_motion(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    #
    # Navigation Helpers
    #

    # @brief Localize at a geometry_msgs/Pose
    def localize(self, pose):
        self.get_logger().info("Localizing robot")
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = pose
        # Smaller covariance since we are in a tight arena
        msg.pose.covariance[0] = 0.0125
        msg.pose.covariance[7] = 0.0125
        msg.pose.covariance[35] = 0.06853891909122467
        while not self._localized_at(pose):
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=1.0)

    # @brief Internal helper
    def _localized_at(self, pose, tol=0.1):
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.pose.orientation.w = 1.0

        try:
            p = self.buffer.transform(p, 'map').pose
        except Exception:
            self.get_logger().error("Transform exception")
            return False

        if abs(p.position.x - pose.position.x) > tol or abs(p.position.y - pose.position.y) > tol:
            return False

        return True

    # @brief Navigate to a geometry_msgs/Pose
    def go_to_pose(self, pose):
        while not self.nav2_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for nav2 action server')

        # Clear any previous result
        self.nav2_result = None

        # Setup goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose

        # Send goal and wait for goal to be accepted or rejected
        self.nav2_goal_future = self.nav2_action_client.send_goal_async(goal)
        self.nav2_goal_future.add_done_callback(self.nav2_goal_callback)

    # @brief NavigateToPose async goal callback
    def nav2_goal_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Nav2 goal was accepted')
            self.nav2_result_future = goal_handle.get_result_async()
            self.nav2_result_future.add_done_callback(self.nav2_result_callback)
        else:
            self.get_logger().warn('Nav2 goal was rejected')
            self.nav2_result = "NOT ACCEPTED"

    # @brief NavigateToPose async result callback
    def nav2_result_callback(self, future):
        self.nav2_result = future.result().status
        print(future.result())

    #
    # Etherbotix IO Helpers
    #

    def enable_fan(self, enable):
        fan_en = 1 if enable else 0
        fan_dir = 1
        self.etherbotix.set_digital_pin(FAN_PIN, fan_en, fan_dir)

    #
    # ROS2 Callbacks
    #

    # @brief Callback for ROI of detected flame (if any)
    def roi_callback(self, msg):
        self.roi = msg

    # @brief Odometry from robot base
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.x
        self.odom_th = atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2.0

    # @brief AMCL pose callback
    def amcl_callback(self, msg):
        self.map_x = msg.pose.pose.position.x
        self.map_y = msg.pose.pose.position.x
        self.map_th = atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2.0


if __name__ == '__main__':
    rclpy.init()
    try:
        node = FirebotStateMachine()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.enable_fan(False)
        node.destroy_node()
        rclpy.shutdown()
