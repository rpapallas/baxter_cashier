#!/usr/bin/python

"""
This class allows two cameras to align with each other by publishing a tf.
Creator: Muhannad, University of Leeds
"""

import rospy
import tf
import numpy as np
import cv2


class Calibrator:
    """Calibrator aligns two camera's POV to a single one."""

    def __init__(self, base_topic, target_topic):
        """Default constructor configuring the class properties."""
        rospy.init_node('camera_calibrator_tf_broadcaster')
        self.rate = rospy.Rate(100.0)

        # Some initialisation
        self.quaternion = None
        self.xyz = [0.0, 0.0, 0.0]
        self.rpy = [0, 0, 0]

        # The class' brodcasters
        self.brodcaster_1 = tf.TransformBroadcaster()

        # Topics that will be used to broadcast tf
        self.base_topic = base_topic
        self.target_topic = target_topic

        # Initialise the trackbars (sliders) for the CV window
        self._create_trackbars_for_window()

    def _create_trackbars_for_window(self):
        self.cv2.namedWindow('image')

        # Create trackbars
        self.cv2.createTrackbar('x', 'image', 0, 3000, self._callback)
        self.cv2.createTrackbar('y', 'image', 0, 3000, self._callback)
        self.cv2.createTrackbar('z', 'image', 0, 3000, self._callback)

        self.cv2.createTrackbar('Roll',  'image', 0, 3600, self._callback)
        self.cv2.createTrackbar('Pitch', 'image', 0, 3600, self._callback)
        self.cv2.createTrackbar('Yaw',   'image', 0, 3600, self._callback)

    def _callback(self, _):
        def apply_formula(value):
            return value * np.pi / 1800

        # Tune xyz and rpy with the current positions of the trackbars
        self.xyz = self._extract_xyz_from_trackbars()
        self.rpy = self._extract_rpy_from_trackbars()

        # Using List Comprehension we get new values for r, p and y
        # based on static formula computed by `_get_new_value_for` static
        # method
        roll, pitch, yaw = map(apply_formula, self.rpy)

        # Using Euler method calculates the quaternion from roll, pitch and yaw
        self.quaternion = tf.transformations.quaternion_from_euler(roll,
                                                                   pitch,
                                                                   yaw)

    def _extract_xyz_from_trackbars(self):
        """ Extracts x, y and z values from CV2 trackbars. """

        x = self.cv2.getTrackbarPos('x', 'image') / 1000.0 - 1
        y = self.cv2.getTrackbarPos('y', 'image') / 1000.0 - 1
        z = self.cv2.getTrackbarPos('z', 'image') / 1000.0 - 1

        return [x, y, z]

    def _extract_rpy_from_trackbars(self):
        """ Extracts r, p and y values from CV2 trackbars. """

        r = self.cv2.getTrackbarPos('Roll', 'image')
        p = self.cv2.getTrackbarPos('Pitch', 'image')
        y = self.cv2.getTrackbarPos('Yaw', 'image')

        return [r, p, y]

    def _print_current_configuration(self, results):
        print "xyz: {} | rpy: {} | q: {}".format(self.xyz,
                                                 self.rpy,
                                                 self.quaternion)

    def calibrate(self):
        """Method performs the basic operation."""

        while not rospy.is_shutdown():
            img = np.zeros((300, 512, 3), np.uint8)
            self.cv2.imshow('image', img)
            _ = self.cv2.waitKey(1) & 0xFF

            self._print_current_configuration()

            self.brodcaster_1.sendTransform(tuple(self.xyz),
                                            tuple(self.quaternion),
                                            rospy.Time.now(),
                                            self.base_topic,
                                            self.target_topic)

            self.rate.sleep()


if __name__ == '__main__':
    calibrator = Calibrator(base_topic="camera_depth_optical_frame",
                            target_topic="torso")
    calibrator.calibrate()
