#!/usr/bin/env python
# cob_body_tracker/user_1/left_elbow
# cob_body_tracker/user_1/left_hand
# cob_body_tracker/user_1/left_shoulder

import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('hand_pose_detector')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/camera_depth_optical_frame', 'cob_body_tracker/user_1/left_hand', rospy.Time(0))
            print str(trans) + ", " + str(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

        rate.sleep()