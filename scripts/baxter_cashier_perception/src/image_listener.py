#!/usr/bin/env python

"""
Based:
http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ImageListener:
    def __init__(self, image_topic):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)
        self.counter = 0
        self.detected = None

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        if self.counter < 5:
            self.counter += 1
            return

        if self.counter == 5:
            print "Now tracking..."
            self.counter += 1

        # The 1 means we want the image in BGR, and not in grayscale.
        img = cv_image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_range_red = np.array([101, 100, 100])
        upper_range_red = np.array([400, 255, 255])

        lower_range_orange = np.array([1, 100, 100])
        upper_range_orange = np.array([101, 255, 255])

        hsv2 = hsv.copy()

        mask_orange = cv2.inRange(hsv, lower_range_orange, upper_range_orange)
        mask_red = cv2.inRange(hsv2, lower_range_red, upper_range_red)

        if np.count_nonzero(mask_red) > 40000:
            if np.count_nonzero(mask_red) > np.count_nonzero(mask_orange):
                if self.detected == "five" or self.detected is None:
                    print "Detected a ONE"
                    self.detected = "one"

        if np.count_nonzero(mask_orange) > 40000:
            if np.count_nonzero(mask_orange) > np.count_nonzero(mask_red):
                if self.detected == "one" or self.detected is None:
                    print "Detected a FIVE"
                    self.detected = "five"


if __name__ == '__main__':
    ic = ImageListener("/camera/rgb/image_rect_color")
    rospy.init_node('image_listener', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    cv2.destroyAllWindows()
