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
'''

# System imports
import argparse
import os
import sys
import time

import baxter_interface
from baxter_interface import CHECK_VERSION

# ROS specific imports
import rospy
import cv2
import cv_bridge
import rospkg

from sensor_msgs.msg import (Image,)

# MoveIt Imports
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import Header

# Project specific imports
from baxter_cashier_manipulation.srv import GetUserPose
from baxter_cashier_manipulation.srv import RecogniseBanknote
from baxter_pose import BaxterPose
from baxter_controller import BaxterPlanner
from moveit_controller import MoveItPlanner

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
        self.planner = MoveItPlanner()

        # TODO: Make this zero, is just for testing purposes set to 3
        self.amount_due = 3
        self.customer_last_pose = None

        # Ensure that the hand calibrated for first time
        print "1. Please move Baxter's right hand above the first banknote."
        raw_input("Click ENTER to set the pose...")
        self.first_banknote_pose = self.planner.get_end_effector_current_pose("right")

    def interact(self):
        """
        Handles the main logic of detecting the entrance of new customer, and
        determining if the next action is to get or give money.
        """

        def pose_is_outdated(created_time):
            return (time.time() - created_time) > 3

        # Do this while customer own money or baxter owns money
        while self.amount_due != 0:
            # Get the hand pose of customer's two hands.
            left_pose, right_pose = self.get_pose_from_space()

            if pose_is_outdated(left_pose.created) and pose_is_outdated(right_pose.created):
                continue

            is_reachable = False
            pose = None
            arm = None

            # If the left pose is not empty (i.e a user is there)
            if not left_pose.is_empty():
                # Verify that Baxter can move there
                is_reachable = self.planner.is_pose_reachable_by_arm(left_pose,
                                                                     self.planner.right_arm)

                if is_reachable:
                    pose = left_pose
                    arm = self.planner.right_arm

            # If the right pose is not empty (i.e a user is there) and left hand didn't work
            if not right_pose.is_empty() and not is_reachable:
                # Verify that Baxter can move there
                is_reachable = self.planner.is_pose_reachable_by_arm(right_pose,
                                                                     self.planner.left_arm)

                if is_reachable:
                    pose = right_pose
                    arm = self.planner.left_arm

            # If we found a reachable pose
            if is_reachable:
                if self.amount_due < 0:  # Baxter owns money
                    self.give_money_to_customer(pose, arm)
                else:  # Customer owns money
                    self.take_money_from_customer(pose)
            else:
                print "Wasn't able to move hand to goal position"

    def take_money_from_customer(self, pose, arm):
        self.planner.move_to_position(pose, arm)

        time.sleep(1)

        # Open/Close the Gripper to catch the money from customer's hand
        self.planner.open_gripper()

        time.sleep(1)

        self.planner.close_gripper()

        time.sleep(1)

        # Moves Baxter hand to head
        self.planner.move_hand_to_head_camera()

        recognised_banknote = self.get_banknote_value()

        if recognised_banknote != -1:
            image = "five_bill_recognised.png" if recognised_banknote == 5 else "one_bill_recognised.png"
            self.show_image_to_baxters_head_screen(image)

            # Since we detected amount, subtract the value from the own amount
            self.amount_due -= int(recognised_banknote)
            self.customer_last_pose = (pose, arm)
            # TODO: Put the banknote to the table
        else:
            self.show_image_to_baxters_head_screen("unable_to_recognise.png")

        self.planner.set_neutral_position_of_limb()

    def get_banknote_value(self):
        # This method blocks until the service 'get_user_pose' is available
        rospy.wait_for_service('recognise_banknote')

        try:
            # Handle for calling the service
            recognise_banknote = rospy.ServiceProxy('recognise_banknote',
                                                    RecogniseBanknote)

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

    def show_image_to_baxters_head_screen(self, image_path):
        rospack = rospkg.RosPack()
        path = rospack.get_path('baxter_cashier_manipulation') + "/img/" + image_path
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
        pub.publish(msg)
        # Sleep to allow for image to be published
        rospy.sleep(1)

    def give_money_to_customer(self, pose):
        if self.amount_due <= -5:
            money_to_give_back = 5
            self.take_a_five_banknote()
        elif self.amount_due >= -4 and self.amount_due <= -1:
            money_to_give_back = 1
            self.take_a_one_banknote()

        pose, baxter_arm = self.customer_last_pose
        self.planner.move_to_position(pose, baxter_arm)

        # Waiting user to reach the robot to get the money
        time.sleep(2)

        self.planner.open_gripper()
        self.amount_due += money_to_give_back

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
            # IMPORTANT: Note that for some reason the Skeelton Tracker library,
            # identifies the left hand as the right and the right as left,
            # hence an easy and quick fix was to request the opposite hand here.
            left_hand = get_user_pose(user_number=1, body_part='right_hand')
            right_hand = get_user_pose(user_number=1, body_part='left_hand')
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

    while True:
        baxter.interact()
