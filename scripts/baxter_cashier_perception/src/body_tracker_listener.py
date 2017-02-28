#!/usr/bin/env python
"""
This script acts as a listener to the Skeleton Tracker provided by the
cob_people_perception library. It listens to a specific frame of the
user's hand and returns the pose to the caller.

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
import os
import rospy
import tf
import time
import subprocess
from baxter_cashier_manipulation.srv import *


class InvalidBodyPartException(Exception):
    """
    Raises exception if the body part string provided by the user is
    not a valid one (i.e not provided by the skeleton tracker)
    """
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return """ The provided body part {} is not a valid body part
               for the skeleton tracker.""".format(self.value)


class BodyTrackerListener:
    def __init__(self):
        # General default configuration for the listener
        self._listener = tf.TransformListener()
        self._RATE = rospy.Rate(0.3)

    def _is_body_part_valid(self, body_part):
        """
        Given a body_part will return true if the part is valid or
        false if is invalid.
        """
        possible_body_parts = ['head', 'neck', 'torso', 'left_shoulder',
                               'right_shoulder', 'left_elbow', 'right_elbow',
                               'left_hand', 'right_hand', 'left_hip',
                               'right_hip', 'left_knee', 'right_knee',
                               'left_foot', 'right_foot']

        return True if body_part in possible_body_parts else False

    # TODO: Consider refactoring the name of the method to make more sense.
    def start_listening_for(self, request):
        """
        Bridge method that starts the Skeleton Tracker, starts the process of
        listening to the pose and kill the skeleton tracker.

        Returns back the transformation and rotation of the pose requested.
        """
        # Throw an exception if body part not valid
        if not self._is_body_part_valid(request.body_part):
            try:
                raise InvalidBodyPartException(request.body_part)
            except InvalidBodyPartException as e:
                print e

        print "Server received: {} and {}".format(request.user_number,
                                                  request.body_part)
        print "Start listenning..."

        tran, rot = self._listen(user_number=request.user_number,
                                 body_part=request.body_part)

        print "Finished listenning."

        return GetUserPoseResponse(tran, rot)

    def _listen(self, user_number, body_part):
        # Source is the node parent and target the child we are looking for.
        source = '/camera_depth_optical_frame'
        target = "cob_body_tracker/user_{}/{}".format(user_number, body_part)

        timeout_start = time.time()
        timeout = 5   # [seconds]

        transformation = [0, 0, 0]
        rotation = [0, 0, 0, 0]

        # while time.time() < timeout_start + timeout:
        #     try:
        #         # Try to listen for the transformation and rotation of the node
        #         (transformation, rotation) = self._listener.lookupTransform(source,
        #                                                                     target,
        #                                                                     rospy.Time(0))
        #     except (tf.LookupException, tf.ConnectivityException,
        #             tf.ExtrapolationException) as e:
        #         print e
        #
        #     self._RATE.sleep()

        # Temp tran and rot for testing purposes
        transformation = [0.657579481614, 0.451981417433, 0.2388352386502]
        rotation = [-0.366894936773, 0.885980397775, 0.108155782462, 0.262162481772]
        return transformation, rotation

    def get_active_users_in_sence(self):
        pass
        # TODO: Implement the functionality here to detect which number of
        # users are in the sence. For example user_1 may not be in sence
        # anymore.


if __name__ == '__main__':
    rospy.init_node("body_tracker_listener", anonymous=True)
    tracker_listener = BodyTrackerListener()

    # Create the so called: "Service Node" of the service.
    # ====================================================
    # - Declare a service named 'get_user_pose' with GetUserPose service type.
    # - All requests to this service are passed to start_listening_for method.
    # - start_listening_for method is called with instances GetUserPoseRequest
    #   and returns instances of GetUserPoseResponse.

    s = rospy.Service('get_user_pose',
                      GetUserPose,
                      tracker_listener.start_listening_for)

    # Keep from exiting until this node is stopped
    rospy.spin()
