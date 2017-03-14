#!/usr/bin/env python

'''
    This file includes functions that computes the Inverse Kinematics
    of a given limb and for the given pose.

    Will also allow moving the corresponding limb's joints given the
    result of the Inverse Kinematics.

    References & Credits:
    1. https://github.com/RethinkRobotics/
        baxter_examples/blob/master/scripts/joint_position_keyboard.py
    2. http://sdk.rethinkrobotics.com/wiki/IK_Service_-_Code_Walkthrough

    Finally API Documentation of Limb class was also useful and can be
    found here:
    1. http://docs.ros.org/groovy/api/baxter_interface/html/
        baxter_interface.limb-pysrc.html
    2. http://api.rethinkrobotics.com/baxter_interface/html/
        baxter_interface.limb.Limb-class.html

    Author: Rafael Papallas
'''

# System imports
import argparse
import sys
import time

import baxter_interface
from baxter_interface import CHECK_VERSION

# ROS specific imports
import rospy
# import rosgraph.masterapi

# MoveIt Imports
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import Header

# Project specific imports
from baxter_cashier_manipulation.srv import GetUserPose
from baxter_cashier_manipulation.srv import RecogniseBanknote
from baxter_pose import BaxterPose
from baxter_controller import BaxterPlanner
from moveit_controller import MoveitPlanner


def planner_factory(planner="baxter"):
    planners = dict(baxter=BaxterPlanner, moveit=MoveitPlanner)
    return planners[planner]()


class Shopkeeper:
    def __init__(self):
        # Initialisation
        rospy.init_node("baxter_cashier")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        rs.enable()

        # This is the camera topic to be used for money recognition (Baxter's
        # head camera or RGB-D camera)
        self._money_recognition_camera_topic = "/cameras/head_camera/image"

        # Baxter's libms configured
        self.planner = planner_factory(planner="baxter")

        # TODO: Make this zero, is just for testing purposes set to 3
        self.amount_due = 3

    def interact(self):
        """
        Handles the main logic of detecting the entrance of new customer, and
        determining if the next action is to get or give money.
        """

        # Do this while customer own money or baxter owns money
        while self.amount_due != 0:
            # Get the hand pose of customer's two hands.
            left_pose, right_pose = self.get_pose_from_space()
            is_reachable = False
            pose = None

            # If the left pose is not empty (i.e a user is there)
            if not left_pose.is_empty():
                # Verify that Baxter can move there
                is_reachable = self.planner.is_pose_reachable_by_robot(left_pose)

                if is_reachable:
                    pose = left_pose

            # If the right pose is not empty (i.e a user is there) and left hand didn't work
            if not right_pose.is_empty() and not is_reachable:
                # Verify that Baxter can move there
                is_reachable = self.planner.is_pose_reachable_by_robot(right_pose)

                if is_reachable:
                    pose = right_pose

            # If we found a reachable pose
            if is_reachable:
                if self.amount_due < 0:  # Baxter owns money
                    self.give_money_to_customer(pose)
                else:  # Customer owns money
                    self.take_money_from_customer(pose)
            else:
                print "Wasn't able to move hand to goal position"

    def take_money_from_customer(self, pose):
        self.planner.move_to_position(pose)

        # Open/Close the Gripper to catch the money from customer's hand
        self.planner.open_gripper()
        self.planner.close_gripper()

        # Moves Baxter hand to head
        self.planner.move_hand_to_head_camera()

        recognised_banknote = self.get_banknote_value()

        if recognised_banknote != -1:
            print "Received: " + str(recognised_banknote)

            # Since we detected amount, subtract the value from the own amount
            self.amount_due -= int(recognised_banknote)
            # TODO: Put the banknote to the table
        else:
            print "Unable to recognise banknote"

    def get_banknote_value(self):
        # This method blocks until the service 'get_user_pose' is available
        rospy.wait_for_service('recognise_banknote')

        try:
            # Handle for calling the service
            recognise_banknote = rospy.ServiceProxy('recognise_banknote', RecogniseBanknote)

            # Use the handle as any other normal function
            value = recognise_banknote(camera_topic=self._money_recognition_camera_topic)
            if value.banknote_amount != -1:
                print value.banknote_amount
            else:
                print "Nothing detected"

            return value.banknote_amount
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return None

    def give_money_to_customer(self, pose):
        if self.amount_due <= -5:
            money_to_give_back = 5
            self.take_a_five_banknote()
        elif self.amount_due >= -4:
            money_to_give_back = 1
            self.take_a_one_banknote()

        self.planner.move_to_position(pose)

        # Waiting user to reach the robot to get the money
        time.sleep(2)

        self.planner.open_gripper()
        self.amount_due += money_to_give_back

    # def get_list_of_users(self):
    #     master = rosgraph.masterapi.Master('/camera_depth_optical_frame')
    #     print master.getPublishedTopics('/cob_body_tracker')

    def get_pose_from_space(self):
        '''
        Returns a pose from space.
        '''
        # This method blocks until the service 'get_user_pose' is available
        rospy.wait_for_service('get_user_pose')

        try:
            # Handle for calling the service
            get_user_pose = rospy.ServiceProxy('get_user_pose', GetUserPose)

            # Use the handle as any other normal function
            left_hand = get_user_pose(user_number=1, body_part='left_hand')
            right_hand = get_user_pose(user_number=1, body_part='right_hand')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        x1, y1, z1 = left_hand.transformation
        x2, y2, z2, w = left_hand.rotation
        left_hand_pose = BaxterPose(x1, y1, z1, x2, y2, z2, w)

        x1, y1, z1 = right_hand.transformation
        x2, y2, z2, w = right_hand.rotation
        right_hand_pose = BaxterPose(x1, y1, z1, x2, y2, z2, w)

        return left_hand_pose, right_hand_pose


if __name__ == '__main__':
    baxter = Shopkeeper()
    # baxter.get_list_of_users()

    while True:
        baxter.interact()
