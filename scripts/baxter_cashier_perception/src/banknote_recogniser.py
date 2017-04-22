#!/usr/bin/env python
"""
Banknote recogniser.

This script acts as a service and is responsible to detect and recognise
banknotes from the given camera.

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

# System specific imports
import time

# ROS specific imports
import rospy
import tf

# Project specific imports
from baxter_cashier_manipulation.srv import RecogniseBanknoteResponse
from baxter_cashier_manipulation.srv import RecogniseBanknote


class BanknoteRecogniser:
    """Banknote recogniser class."""

    def __init__(self):
        """Default constructor."""
        self._listener = tf.TransformListener()
        self._RATE = rospy.Rate(0.3)

    def try_to_detect(self, amount):
        """
        Will try to detect the given amount from the tf topic.

        Will return the amount given if the banknote was detected or will
        return None if the given amount was not detected.
        """
        try:
            _ = self._listener.lookupTransform("base",
                                               "ar_marker_{}".format(amount),
                                               rospy.Time(0))

            return amount
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            print(e)

        return None

    def detect(self, request):
        """Will return the amount detected or -1 if nothing detected."""
        timeout_start = time.time()
        timeout = 5   # [seconds]

        while time.time() < timeout_start + timeout:
            # Try to detect either the 5 or the 1 banknote.
            if self.try_to_detect(5) is not None:
                return RecogniseBanknoteResponse(5)
            elif self.try_to_detect(1) is not None:
                return RecogniseBanknoteResponse(1)

            self._RATE.sleep()

        # If time over and nothing returned, nothing detected and return -1
        return RecogniseBanknoteResponse(-1)


if __name__ == '__main__':
    print("Starting the service ...")

    # Create the service
    rospy.init_node("bank_note_recogniser", anonymous=True)
    banknote_recogniser = BanknoteRecogniser()

    s = rospy.Service('recognise_banknote',
                      RecogniseBanknote,
                      banknote_recogniser.detect)

    rospy.spin()
