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
import threading
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


class Banknote:
    def __init__(self, pose):
        self.pose = pose
        self.is_available = True


class BanknotesOnTable:
    def __init__(self, initial_pose, table_side, num_of_remaining_banknotes):
        self._table_side = table_side
        self._initial_pose = initial_pose
        self._number_of_remaining_banknotes = num_of_remaining_banknotes
        self.banknotes = [Banknote(initial_pose)]

        self._calculate_pose_of_remaining_poses(self)

    def is_left(self):
        return True if self._table_side == "left" else False

    def is_right(self):
        return True if self._table_side == "right" else False

    def reset_availability_for_all_banknotes(self):
        for banknote in self.banknotes:
            banknote.is_available = True

    def get_next_available_banknote(self):
        for banknote in self.banknotes:
            if banknote.is_available:
                banknote.is_available = False
                return banknote

        return None

    def _calculate_pose_of_remaining_poses(self):
        static_x_to_be_added = 0.05  # 5cm

        for _ in range(0, self._number_of_remaining_banknotes):
            new_pose = self.banknotes[-1].transformation_x + static_x_to_be_added
            self.banknotes.append(Banknote(new_pose))


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

        self.banknotes_table_left = self.set_banknotes_on_table_for_side("left")
        self.banknotes_table_right = self.set_banknotes_on_table_for_side("right")

    def set_banknotes_on_table_for_side(self, side):
        """
        Records and calculates the poses of the banknotes on the table.

        This function will ask two questions from the user:
        (1) To move Baxter's arm to the position of the first banknote.
        (2) The number of the remaining banknotes on the table.

        It will then record the pose and also calculate the poses of the \
        remaining banknotes.

        This function will also move Baxter's arms to the remaining banknotes
        just to ensure that the remaining banknotes are placed correctly.

        Finally the returned list of poses will include a list tuple
        (pose, bool) for every banknote on the table of either left or right
        side. The `pose` is the pose of the corresponding banknote, where the
        bool value (initially set to True) represents if Baxter have used this
        banknote.
        """
        arm = self.planner.left_arm if side == "left" else self.planner.right_arm

        print "=============================================================="
        print " Calibrating poses of banknotes of the table's side: " + side
        print "=============================================================="

        print "1. Please move Baxter's right hand above the first banknote."
        raw_input("Press ENTER to set the pose...")
        initial_pose = self.planner.get_end_effector_current_pose(side)

        # Calculate the remaining poses
        num = int(raw_input("2. Number of REMAINING banknotes on this side of the table? : "))

        banknotes_on_table = BanknotesOnTable(initial_pose=initial_pose,
                                              table_side=side,
                                              num_of_remaining_banknotes=num)

        for banknote in banknotes_on_table.banknotes[1:]:
            self.planner.move_to_position(banknote.pose, arm)

        self.planner.active_hand = arm
        self.planner.set_neutral_position_of_limb()

        return banknotes_on_table

    def interact(self):
        """
        Handles the main logic of detecting the entrance of new customer, and
        determining if the next action is to get or give money.
        """

        def pose_is_outdated(pose):
            """Checks whether the pose is recent or not."""
            return (time.time() - pose.created) > 3

        self.show_eyes_normal()

        # Do this while customer own money or baxter owns money
        while self.amount_due != 0:

            # If the amount due is negative, Baxter owns money
            if self.amount_due < 0:
                self.give_money_to_customer()
                continue

            # Get the hand pose of customer's two hands.
            left_pose, right_pose = self.get_pose_from_space()

            # If the pose detected is not too recent, ignore.
            if pose_is_outdated(left_pose) and pose_is_outdated(right_pose):
                continue

            # NOTE that we use right hand for left pose and left hand for right
            # pose. Baxter's left arm is closer to user's right hand and vice
            # versa.
            if self.pose_is_reachable(left_pose, "right"):
                self.take_money_from_customer(left_pose,
                                              self.planner.right_arm)

            elif self.pose_is_reachable(right_pose, "left"):
                self.take_money_from_customer(right_pose,
                                              self.planner.right_arm)

            else:
                print "Wasn't able to move hand to goal position"

    def pose_is_reachable(self, pose, side):
        arm = self.planner.left_arm if side == "left" else self.planner.right_arm

        if not left_pose.is_empty():
            # Verify that Baxter can move there
            is_reachable = self.planner.is_pose_reachable_by_arm(pose, arm)
            return is_reachable

        return False

    def take_money_from_customer(self, pose, arm):
        self.planner.move_to_position(pose, arm)

        # Open/Close the Gripper to catch the money from customer's hand
        self.planner.open_gripper()
        rospy.sleep(1)
        self.planner.close_gripper()
        rospy.sleep(1)

        # Moves Baxter hand to head
        self.planner.move_hand_to_head_camera()

        # Here show Baxter's eyes moving to show that the robot is not stuck
        # but is instead "thinking" (because eyes are moving)
        self.run_nonblocking(self.make_eyes_animated_reading_banknote)

        # Start reading the banknote value
        banknote_value = self.get_banknote_value()

        if banknote_value != -1:
            # Show image of the recognised banknote.
            image = "five_bill_recognised.png" if banknote_value == 5 else "one_bill_recognised.png"
            self.show_image_to_baxters_head_screen(image)

            # Since we detected amount, subtract the value from the own amount
            self.amount_due -= int(banknote_value)
            self.customer_last_pose = (pose, arm)
            self.planner.leave_banknote_to_the_table()
            rospy.sleep(1)
        else:
            self.show_image_to_baxters_head_screen("unable_to_recognise.png")

        self.show_eyes_normal()
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

    def run_nonblocking(self, function):
        thread = threading.Thread(target=function)
        thread.start()

    def make_eyes_animated_reading_banknote(self):
        self.show_eyes_focusing()
        self.show_eyes_focusing_right()
        self.show_eyes_focusing_left()
        self.show_eyes_focusing_right()
        self.show_eyes_focusing_left()

    def show_eyes_normal(self):
        self.show_image_to_baxters_head_screen("normal_eyes.png")

    def show_eyes_focusing(self):
        self.show_image_to_baxters_head_screen("looking_eyes.png")

    def show_eyes_focusing_left(self):
        self.show_image_to_baxters_head_screen("looking_left_eyes.png")

    def show_eyes_focusing_right(self):
        self.show_image_to_baxters_head_screen("looking_right_eyes.png")

    def show_image_to_baxters_head_screen(self, image_path):
        rospack = rospkg.RosPack()
        path = rospack.get_path('baxter_cashier_manipulation') + "/img/" + image_path
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=20)
        pub.publish(msg)
        # Sleep to allow for image to be published
        rospy.sleep(1)

    def pick_banknote_from_table(self, arm):
        # TODO: Make sure that this won't happen again by passing the arm in gripper open/close
        self.planner.active_hand = arm
        self.planner.open_gripper()
        pose = None

        if arm.is_left():
            banknote = self.banknotes_table_left.get_next_available_banknote()
        elif arm.is_right():
            banknote = self.banknotes_table_right.get_next_available_banknote()

        if banknote is not None:
            self.planner.move_to_position(banknote.pose, arm)
            rospy.sleep(1)
            self.planner.close_gripper()
            self.planner.set_neutral_position_of_limb()
        else:
            print "No available banknotes on the table..."

    def give_money_to_customer(self):
        customer_hand_pose, baxter_arm = self.customer_last_pose
        money_to_give_back = 1

        self.pick_banknote_from_table(arm)
        self.planner.move_to_position(customer_hand_pose,
                                      arm)

        # Waiting user to reach the robot to get the money
        rospy.sleep(2)
        self.planner.open_gripper()
        self.amount_due += money_to_give_back

        if self.amount_due >= 0:
            self.planner.set_neutral_position_of_limb()


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
