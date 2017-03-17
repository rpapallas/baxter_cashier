#!/usr/bin/env python

"""
    Copyright (C)  2016/2017 The University of Leeds and Rafael Papallas

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from baxter_cashier_manipulation.srv import *
import time
from matplotlib import pyplot as plt
import tf


class BanknoteRecogniser:
    def __init__(self):
        self._listener = tf.TransformListener()
        self._RATE = rospy.Rate(0.3)

    def detect(self, request):
        found = -1
        timeout_start = time.time()
        timeout = 5   # [seconds]

        while found is None or (time.time() < timeout_start + timeout):
            try:
                (transformation, _) = self._listener.lookupTransform("base",
                                                                     "ar_marker_5",
                                                                     rospy.Time(0))
                found = 5
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                print e

            try:
                (transformation, _) = self._listener.lookupTransform("base",
                                                                     "ar_marker_1",
                                                                     rospy.Time(0))
                return 1
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                print e

            self._RATE.sleep()

        return RecogniseBanknoteResponse(found)


if __name__ == '__main__':
    print "Starting.."
    rospy.init_node("bank_note_recogniser", anonymous=True)
    banknote_recogniser = BanknoteRecogniser()

    s = rospy.Service('recognise_banknote',
                      RecogniseBanknote,
                      banknote_recogniser.detect)

    rospy.spin()
