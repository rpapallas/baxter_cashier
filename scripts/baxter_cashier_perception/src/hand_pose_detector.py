#!/usr/bin/env python
"""
This script acts as a listener to the Skeleton Tracker provided by the
cob_people_perception library. It listens to a specific frame of the
user's hand and returns the pose to the caller.

It uses threads to execute the job so it does not hold other code calling
this class.

The cob_people_perception library publishes the tf frames in the following
manner:
- Parent: /camera_depth_optical_frame
- Childs: /cob_body_tracker/user_n/{x}
  - Where n is the number of the user (1, 2, 3 etc)
  - Where x is the body part with possible options:
    - Shoulders: left_shoulder, right_shoulder
    - Elbows: left_elbow, right_elbow
    - Hands: left_hand, right_hand
    - Hips: left_hip, right_hip
    - Knees: left_knee, right_knee
    - Feet: left_foot, right_foot
    - Other: head, neck, torso
"""

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
