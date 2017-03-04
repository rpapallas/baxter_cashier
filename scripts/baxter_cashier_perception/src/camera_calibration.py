#!/usr/bin/python

"""
Source code credits by Muhannad, University of Leeds
https://github.com/OMARI1988/baxter_pykdl/blob/master/scripts/kinect_frame.py
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

        self.q = None
        self.xyz = [0.0, 0.0, 0.0]
        self.q_calib1 = [-0.62690571,  0.63143915, -0.31687864,  0.32842314]
        self.xyz_calib1 = [0.272, -0.03600000000000003, 0.23399999999999999]
        self.rpy = [0, 0, 0]
        self.rpy_calib = [2343, 10, 2701]  # the values you should use
        self.rate = rospy.Rate(100.0)

        self.brodcaster_1 = tf.TransformBroadcaster()
        self.brodcaster_2 = tf.TransformBroadcaster()

        self.base_topic = base_topic
        self.target_topic = target_topic

    def calibrate(self):
        """Method performs the basic operation."""
        def nothing():
            # get current positions of four trackbars
            self.xyz[0] = cv2.getTrackbarPos('x', 'image') / 1000.0-1
            self.xyz[1] = cv2.getTrackbarPos('y', 'image') / 1000.0-1
            self.xyz[2] = cv2.getTrackbarPos('z', 'image') / 1000.0-1

            r = cv2.getTrackbarPos('Roll', 'image')
            p = cv2.getTrackbarPos('Pitch', 'image')
            y = cv2.getTrackbarPos('Yaw', 'image')

            self.rpy = [r, p, y]

            pi = np.pi
            self.q = tf.transformations.quaternion_from_euler(r * pi / 1800,
                                                              p * pi / 1800,
                                                              y * pi / 1800)

        # img = np.zeros((300, 512, 3), np.uint8)
        cv2.namedWindow('image')

        # create trackbars for color change
        cv2.createTrackbar('x', 'image', 0, 3000, nothing)
        cv2.createTrackbar('y', 'image', 0, 3000, nothing)
        cv2.createTrackbar('z', 'image', 0, 3000, nothing)
        cv2.createTrackbar('Roll', 'image', 0, 3600, nothing)
        cv2.createTrackbar('Pitch', 'image', 0, 3600, nothing)
        cv2.createTrackbar('Yaw', 'image', 0, 3600, nothing)

        while not rospy.is_shutdown():
            self.brodcaster_1.sendTransform(tuple(self.xyz_calib1),
                                            tuple(self.q_calib1),
                                            rospy.Time.now(),
                                            self.base_topic,
                                            self.target_topic)
            self.rate.sleep()


if __name__ == '__main__':
    calibrator = Calibrator(base_topic="kinect2_1_ir_optical_frame",
                            target_topic="torso")
    calibrator.calibrate()
