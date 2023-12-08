#!/usr/bin/env python3

import cv2
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, RegionOfInterest

# FLIR Lepton 3.5
# Datasheet specifies that resolution is 0.01C and,
# a pixel value of 30000 = 300K (26.85C). So:
#   0C = 30000 - (26.85 / 0.01)
DEGREES_0C = 27315


class LeptonDriver(Node):

    MODE_RAW_TEMP = 0
    MODE_COLOR = 1

    def __init__(self):
        super().__init__('lepton_driver')

        # TODO: load this from parameter
        self.mode = self.MODE_RAW_TEMP

        self.cv_bridge = cv_bridge.CvBridge()
        self.img_pub = self.create_publisher(Image, 'image', 1)
        self.region_pub = self.create_publisher(RegionOfInterest, 'region', 1)

        self.camera = cv2.VideoCapture('/dev/video0')
        if self.mode == self.MODE_RAW_TEMP:
            # From https://flir.custhelp.com/app/answers/detail/a_id/3387/~/boson-video-and-image-capture-using-opencv-16-bit-y16 # noqa
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', '1', '6', ' '))
            self.camera.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        self.timer = self.create_timer(0.1, self.callback)

    def callback(self):
        ret, frame = self.camera.read()
        if frame is None:
            # TODO: recover from this
            self.get_logger.warn('Unable to get image')
            return

        if self.mode == self.MODE_RAW_TEMP:
            # Remove values below 0C
            frame[frame < DEGREES_0C] = DEGREES_0C

            # Convert image from 0.01 Kelvin to 0.01 Celsius
            frame = frame - DEGREES_0C

            image = self.cv_bridge.cv2_to_imgmsg(frame, 'mono16')

            # Now look for fire (anything above 50C)
            FIRE_CUTOFF = 50 * 100
            candle_pix = np.transpose((frame > FIRE_CUTOFF).nonzero())
            num_candle_pix = len(candle_pix)
            if num_candle_pix == 0:
                self.get_logger().info('No candle found')
            else:
                # Compute region of interest
                x_values = [x for x, y in candle_pix]
                y_values = [y for x, y in candle_pix]
                min_x = min(x_values)
                max_x = max(x_values)
                min_y = min(y_values)
                max_y = max(y_values)

                # Publish region of interest
                msg = RegionOfInterest()
                msg.x_offset = int(min_x)
                msg.y_offset = int(min_y)
                msg.height = int(max_y - min_y)
                msg.width = int(max_x - min_x)
                self.region_pub.publish(msg)

        else:
            # Publish the color image
            image = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')

        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'lepton'
        self.img_pub.publish(image)

    def __del__(self):
        self.camera.release()


if __name__ == '__main__':
    rclpy.init()
    node = LeptonDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
