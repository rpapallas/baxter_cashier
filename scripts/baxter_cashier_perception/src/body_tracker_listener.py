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
import tf
from threading import Thread
import time

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
        self.listen_thread = None

        # These two will be updated when listen_for method is called.
        self.transformation = None
        self.rotation = None

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
        # Throw an exception if body part not valid
        if not self._is_body_part_valid(body_part):
            try:
                raise InvalidBodyPartException(body_part)
            except InvalidBodyPartException as e:
                print e

        self.listen_thread = threading.Thread(target=self._listen, args=(user_number, body_part))

        # By setting them as daemon threads, you can let them run and forget
        # about them, and when your program quits, any daemon threads
        # are killed automatically.
        thread.daemon = True
        thread.start()

    def _listen(self, user_number, body_part):
        # Source is the node parent and target the child we are looking for.
        source = '/camera_depth_optical_frame'
        target = "cob_body_tracker/user_{}/{}".format(user_number, body_part)

        while not rospy.is_shutdown():
            try:
                # Try to listen for the transformation and rotation of the node
                (transformation, rotation) = _listener.lookupTransform(source, target, rospy.Time(0))
                self.transformation = transformation
                self.rotation = rotation
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException) as e:
                print e

            _RATE.sleep()

if __name__ == '__main__':
    tracker_listener = BodyTrackerListener()
    tracker_listener.start_listening_for(user_number=1, body_part="left_hand")

    time.sleep(10)

    if tracker_listener.transformation is not None and tracker_listener.rotation is not None:
        print "Transformation: {}".format(tracker_listener.transformation)
        print "Rotation: {}".format(tracker_listener.rotation)
    else:
        print "Nothing listened."
