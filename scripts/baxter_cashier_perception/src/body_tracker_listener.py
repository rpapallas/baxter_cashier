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
import rospy
import tf
import time
from subprocess import call
from subprocess import Popen


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
        rospy.init_node("body_tracker_listener")
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

    def start_listening_for(self, user_number, body_part, sec_to_listen):
        """
        Bridge method that starts the Skeleton Tracker, starts the process of
        listening to the pose and kill the skeleton tracker.

        Returns back the transformation and rotation of the pose requested.
        """
        # Throw an exception if body part not valid
        if not self._is_body_part_valid(body_part):
            try:
                raise InvalidBodyPartException(body_part)
            except InvalidBodyPartException as e:
                print e

        print "Starting the skeleton tracker..."
        self._start_skeleton_tracker()
        print "Skeleton tracker started"

        # Give some time for the Skeleton Tracker to start normally
        time.sleep(5)

        print "Start listenning..."
        tran, rot = self._listen(user_number=user_number=, body_part=body_part)
        print "Finished listenning."

        print "Killing now the Skeleton Tracker..."
        self._kill_skeleton_tracker()
        print "Skeleton tracker killed."

        return tran, rot


    def _start_skeleton_tracker(self):
        """
        To start the skeleton tracker we need first to start the OpenNI2 node
        that actually starts the camera sensor and then the COB library that
        implements the Skeleton Tracker. These two collaborate together to
        achieve at the end the "Skeleton Tracking".

        We do start the skeleton tracker using Python's Popen.
        """
        self._openni2_node = subprocess.Popen(["roslaunch",
                                               "openni2_launch",
                                               "openni2.launch",
                                               "depth_registration:=true"
                                               ])

        self._skeleton_node = subprocess.Popen(["roslaunch",
                                                "cob_openni2_tracker",
                                                "body_tracker_nodelet.launch"])

    def _kill_skeleton_tracker(self):
        """
        Skeleton Tracker consists of two main processes that both needs to be
        killed when Skeleton Tracker needs to be stopped.
        """
        self._skeleton_node.kill()
        self._openni2_node.kill()

    def _listen(self, user_number, body_part):
        # Source is the node parent and target the child we are looking for.
        source = '/camera_depth_optical_frame'
        target = "cob_body_tracker/user_{}/{}".format(user_number, body_part)

        timeout_start = time.time()
        timeout = 5   # [seconds]

        transformation = [0, 0, 0]
        rotation = [0, 0, 0, 0]

        while time.time() < timeout_start + timeout:
            try:
                # Try to listen for the transformation and rotation of the node
                (transformation, rotation) = _listener.lookupTransform(source,
                                                                       target,
                                                                       rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                print e

            _RATE.sleep()

        return transformation, rotation


if __name__ == '__main__':
    tracker_listener = BodyTrackerListener()
    tracker_listener.start_listening_for(user_number=1, body_part="left_hand")

    time.sleep(10)

    if tracker_listener.transformation is not None \
       and tracker_listener.rotation is not None:
        print "Transformation: {}".format(tracker_listener.transformation)
        print "Rotation: {}".format(tracker_listener.rotation)
    else:
        print "Nothing listened."
