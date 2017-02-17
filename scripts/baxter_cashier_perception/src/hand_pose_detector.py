#!/usr/bin/env python
# cob_body_tracker/user_1/left_elbow
# cob_body_tracker/user_1/left_hand
# cob_body_tracker/user_1/left_shoulder

import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('hand_pose_detector')

    listener = tf.TransformListener()

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/camera_depth_optical_frame', 'cob_body_tracker/user_1/left_hand', rospy.Time(0))
            print str(trans) + ", " + str(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()