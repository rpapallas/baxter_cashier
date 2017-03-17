#!/usr/bin/env python
"""
Baxter Controller.

This  controller  bridge  Baxter IK Solver and planner into class to be used in
other scripts like shopkeeper.py. Note that this is has been replaced by MoveIt
since MoveIt provides a more flexible planenr.  Is only  here as a legacy code.

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

import time
import rospy

# Baxter specific imports
import baxter_interface
from baxter_interface import Gripper
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)


class BaxterArm():
    """Represents a Baxter Arm."""

    def __init__(self, side_name):
        """Initialise the arm with some useful configuration."""
        # The string side_name is used for representation purposes (ie __str__)
        self._side_name = side_name

        # Initialise the limb and gripper of the arm
        self.limb = baxter_interface.Limb(side_name)
        self.gripper = Gripper(side_name, CHECK_VERSION)

        # Set the speed of the arm
        self.limb.set_joint_position_speed(0.1)

        # Calibrate the gripper
        self.gripper.calibrate()

    def __str__(self):
        """
        String  representation  of the  arm.

        Either  'left'  or  'right'  string  will be returned. Useful  when the
        string representation of the arm is needed, like when  accessing the IK
        solver.
        """
        return self._side_name

    def is_left(self):
        """Will return true if this is the left arm, fale otherwise."""
        return True if self._side_name == "left" else False

    def is_right(self):
        """Will return true if this is the right arm, fale otherwise."""
        return True if self._side_name == "right" else False


class BaxterPlanner:
    """
    Baxter's Default Planner and IK Solver.

    This planner will wrap common methods and functions that are needed to plan
    and move Baxter.This includes the IK Solver, moving Baxter to common static
    poses and so on.
    """

    def __init__(self):
        """Will create the two arms of Baxter and configure the active_hand."""
        self.left_arm = BaxterArm("left")
        self.right_arm = BaxterArm("right")

        # Active hand is  used to  keep a state of  which hand  has recently be
        # moved  from the  planenr. For  example, a  user send  request to this
        # script to reach a pose, the algorithms in  this script will determine
        # how to move  there but is not the user t hat will specify which hand,
        # the algorithm here will try both and find which one, hence by keeping
        # an active hand, we are able to use  it for other operaitons like open
        # and close of the gripper on that hand.
        self.active_hand = None

    def is_pose_reachable_by_robot(self, baxter_pose):
        """
        Will indicate if pose is reachable.

        Will return True if either hand can reach the pose, false otherwise.
        """
        baxter_arm, solution = self.ik_solver(baxter_pose.get_pose_stamped())
        return True if solution is not None else False

    def open_gripper(self):
        """
        Will perform 'open' command to Baxter's gripper.

        Note that this will use the active hand, which is set when Baxter's
        arm is moved in a previous execution.
        """
        self.active_hand.gripper.open()
        time.sleep(1)

    def close_gripper(self):
        """
        Will perform 'close' command to Baxter's gripper.

        Note that this will use the active hand, which is set when Baxter's
        arm is moved in a previous execution.
        """
        self.active_hand.gripper.close()
        time.sleep(1)

    def move_hand_to_head_camera(self):
        """Will move Baxter's active hand to head."""
        # These are static joint  configurations that  lead Baxter's hand to be
        # in a money  detection pose (i.e  the hand is  locating  near Baxter's
        # head camera trying to identify the banknote)
        left_hand = {'left_w0': 2.64343239272,
                     'left_w1': -0.846373899716,
                     'left_w2': -2.14527213186,
                     'left_e0': -2.40988381777,
                     'left_e1': 2.12494688642,
                     'left_s0': 0.956820516444,
                     'left_s1': -0.369305874683}

        right_hand = {'right_s0': -0.852509822867,
                      'right_s1': -0.521936963078,
                      'right_w0': 0.557218521199,
                      'right_w1': 0.93572828061,
                      'right_w2': -0.926524395883,
                      'right_e0': 2.33395176877,
                      'right_e1': 1.99149055787}

        config = left_hand if self.active_hand is self.left_arm else right_hand

        self.active_hand.limb.move_to_joint_positions(config)

    def move_to_position(self, baxter_pose):
        """
        Will move Baxter hand to the pose.

        Note that the Baxter's arm that  will be used to move is not specified.
        The  algorithm  will  try  both  and  plan  the  first  one to succeed.
        """
        baxter_arm, solution = self.ik_solver(baxter_pose.get_pose_stamped())

        if solution is not None:
            baxter_arm.limb.move_to_joint_positions(solution)
            self.active_hand = baxter_arm
        else:
            self.active_hand = None

    def set_neutral_position_of_limb(self, arm):
        """Set limb's neutral position."""
        arm.limb.move_to_neutral()

    def ik_solver(self, pose_stamped, arm=None):
        """
        Will perform Inverse Kinematic on a given limb and pose.

        Given the limb and a target pose, will calculate the joint
        configuration of the limb.
        """
        if arm is None:
            arm = self.left_arm

        ns = "ExternalTools/" + str(arm) + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        ikreq.pose_stamp.append(pose_stamped)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))

        if (resp.isValid[0]):
            print("Valid Joint Solution Found")

            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name,
                                   resp.joints[0].position))
            return arm, limb_joints
        else:
            if arm.is_left():
                # If not solution found, try to solve it with the opossite hand
                self.ik_solver(pose_stamped, self.right_arm)
            else:
                print("No solution found.")

        return None, None
