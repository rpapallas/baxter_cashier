#!/usr/bin/env python

"""
Based: http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
"""

import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import imutils
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

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        if self.counter < 5:
            self.counter += 1
            return
        else:
            self.image_sub.unregister()

        # The 1 means we want the image in BGR, and not in grayscale.
        img = cv2.imread(sys.argv[1], 1)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # For Five bill (green)
        lower_range_green = np.array([48, 100, 100], dtype=np.uint8)
        upper_range_green = np.array([68, 255, 255], dtype=np.uint8)

        # For One bill (blue)
        lower_range_blue = np.array([1, 100, 100], dtype=np.uint8)
        upper_range_blue = np.array([21, 255, 255], dtype=np.uint8)

        mask_green = cv2.inRange(hsv, lower_range_green, upper_range_green)
        mask_blue = cv2.inRange(hsv, lower_range_blue, upper_range_blue)

        cv2.imshow('mask green',mask_green)
        cv2.imshow('mask blue', mask_blue)
        cv2.imshow('image', img)

        while(1):
          k = cv2.waitKey(0)
          if(k == 27):
            break

        cv2.destroyAllWindows()

if __name__ == '__main__':
    ic = ImageListener("/camera/rgb/image_rect_color")
    rospy.init_node('image_listener', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    cv2.destroyAllWindows()
