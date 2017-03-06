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
from baxter_cashier_manipulation.srv import RecogniseBanknote


class BanknoteRecogniser:
    def __init__(self):
        self.bridge = CvBridge()
        self.detected_amount = None

    def detect(self, request):
        self.image_sub = rospy.Subscriber(request.camera_topic,
                                          Image,
                                          self.callback)

        while True:
            if self.detected_amount is not None:
                self.image_sub.unregister()
                response = RecogniseBanknoteResponse(self.detected_amount)
                self.detected_amount = None
                return response

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        img = cv_image
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_range_red = np.array([101, 100, 100])
        upper_range_red = np.array([400, 255, 255])

        lower_range_orange = np.array([1, 100, 100])
        upper_range_orange = np.array([101, 255, 255])

        mask_orange = cv2.inRange(hsv, lower_range_orange, upper_range_orange)
        mask_red = cv2.inRange(hsv, lower_range_red, upper_range_red)

        if np.count_nonzero(mask_red) > 40000 or np.count_nonzero(mask_orange) > 40000:
            if np.count_nonzero(mask_red) > np.count_nonzero(mask_orange):
                print "Detected a ONE"
                self.detected_amount = 1

            if np.count_nonzero(mask_orange) > np.count_nonzero(mask_red):
                print "Detected a FIVE"
                self.detected_amount = 5


if __name__ == '__main__':
    banknote_recogniser = BanknoteRecogniser()

    rospy.init_node('bank_note_recogniser', anonymous=True)

    s = rospy.Service('recognise_banknote',
                      RecogniseBanknote,
                      banknote_recogniser.detect)

    rospy.spin()