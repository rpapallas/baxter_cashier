#!/usr/bin/env python
# cob_body_tracker/user_1/left_elbow
# cob_body_tracker/user_1/left_hand
# cob_body_tracker/user_1/left_shoulder



import roslib
roslib.load_manifest('hand_pose_detector')
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('hand_pose_detector')

    # The tf package provides an implementation of a tf.TransformListener
    # to help make the task of receiving transforms easier.
    listener = tf.TransformListener()

    # Here, we create a tf.TransformListener object. Once the listener is
    # created, it starts receiving tf transformations over the wire, and
    # buffers them for up to 10 seconds.
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (transformation, rotation) = listener.lookupTransform('')
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "An exception occurred"

        rate.sleep()
