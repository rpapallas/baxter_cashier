#!/usr/bin/env python
"""
Body Tracker listener.

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
from baxter_cashier_manipulation.srv import GetUserPose
from baxter_cashier_manipulation.srv import GetUserPoseResponse


class InvalidBodyPartException(Exception):
    """
    Invalid Body Part Exception.

    Raises exception if the body part string provided by the user is
    not a valid one (i.e not provided by the skeleton tracker)
    """

    def __init__(self, value):
        """Default constructor accepting the value given."""
        self.value = value

    def __str__(self):
        """String representation of the exception."""
        return """ The provided body part {} is not a valid body part
               for the skeleton tracker.""".format(self.value)


class BodyTrackerListener:
    """
    Body Tracker Listener class.

    Will listen to the cob_openni2_tracker library for poses and will return
    the pose on request.
    """

    def __init__(self):
        """Default constructor."""
        # General default configuration for the listener
        self._listener = tf.TransformListener()
        self._RATE = rospy.Rate(0.1)

    def _is_body_part_valid(self, body_part):
        """
        Will check if body part is valid.

        Given a body_part will return true if the part is valid or
        false if is invalid.
        """
        possible_body_parts = ['head', 'neck', 'torso', 'left_shoulder',
                               'right_shoulder', 'left_elbow', 'right_elbow',
                               'left_hand', 'right_hand', 'left_hip',
                               'right_hip', 'left_knee', 'right_knee',
                               'left_foot', 'right_foot']

        return True if body_part in possible_body_parts else False

    def start_listening_for(self, request):
        """
        Will listen for request.

        Bridge method that starts the Skeleton Tracker, starts the process of
        listening to the pose and kill the skeleton tracker.

        Returns back the transformation and rotation of the pose requested.
        """
        # Throw an exception if body part not valid
        if not self._is_body_part_valid(request.body_part):
            try:
                raise InvalidBodyPartException(request.body_part)
            except InvalidBodyPartException as e:
                print(e)

        print("Server received: {} and {}".format(request.user_number,
                                                  request.body_part))

        tran, rot = self._listen(user_number=request.user_number,
                                 body_part=request.body_part)

        return GetUserPoseResponse(tran, rot)

    def _listen(self, user_number, body_part):
        # Source is the node parent and target the child we are looking for.
        source = '/base'

        trans = [0, 0, 0]
        rotation = [0, 0, 0, 0]

        number_of_consecutive_frames = 0
        timeout_start = time.time()
        timeout = 2   # [seconds]

        while True:
            target = "cob_body_tracker/user_{}/{}".format(user_number, body_part)
            if number_of_consecutive_frames == 1:
                break

            try:
                # Try to listen for the transformation and rotation of the node
                (trans, _) = self._listener.lookupTransform(source,
                                                            target,
                                                            rospy.Time(0))
            except:
                return trans, rotation

            x, y, z = trans

            if number_of_consecutive_frames == 0:
                xs = map(lambda v: v + x, [0.10, -0.10])
                ys = map(lambda v: v + y, [0.10, -0.10])
                zs = map(lambda v: v + z, [0.10, -0.10])

            if (x >= xs[1] and x <= xs[0]) and \
                (y >= ys[1] and y <= ys[0]) and \
                (z >= zs[1] and z <= zs[0]):
                number_of_consecutive_frames += 1
            else:
                number_of_consecutive_frames = 0

            time.sleep(0.5)

        rotation = [0.559, -0.504, 0.480, -0.451]

        if body_part == "right_hand":
            rotation = [-0.513, 0.520, -0.499, 0.467]

        return trans, rotation


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
