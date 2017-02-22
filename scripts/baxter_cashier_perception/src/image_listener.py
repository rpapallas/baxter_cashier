#!/usr/bin/env python

"""
Based:
http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import sys

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # initialize the shape name and approximate the contour
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        # if the shape is a triangle, it will have 3 vertices
        if len(approx) == 3:
            shape = "triangle"

        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        elif len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"

        # if the shape is a pentagon, it will have 5 vertices
        elif len(approx) == 5:
            shape = "pentagon"

        # otherwise, we assume the shape is a circle
        else:
            shape = "circle"

        # return the name of the shape
        return shape


class ImageListener:
    def __init__(self, image_topic):
        # self.image_pub = rospy.Publisher(image_topic, Image)
        self.bridge = CvBridge()
        time.sleep(10)
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
        # else:
        #     self.image_sub.unregister()
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
        if np.count_nonzero(mask_orange) > 40000 or np.count_nonzero(mask_red) > 40000:
            if np.count_nonzero(mask_orange) > np.count_nonzero(mask_red):
                if self.detected == "one" or self.detected is None:
                    print "Detected a FIVE"
                    self.detected = "five"
            elif np.count_nonzero(mask_red) > np.count_nonzero(mask_orange):
                if self.detected == "five" or self.detected is None:
                    print "Detected a ONE"
                    self.detected = "one"

        # cv2.imshow('Five Pounds Detected', mask_orange)
        # cv2.imshow('One Pounds Detected', mask_red)
        # cv2.imshow('image', img)
        #
        # while(1):
        #     k = cv2.waitKey(0)
        #     if(k == 27):
        #         break
        #
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    ic = ImageListener("/camera/rgb/image_rect_color")
    rospy.init_node('image_listener', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    cv2.destroyAllWindows()
