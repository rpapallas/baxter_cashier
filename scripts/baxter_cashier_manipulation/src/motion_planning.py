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

# Baxter specific imports
import baxter_interface
from baxter_interface import Gripper

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

# ROS specific imports
import rospy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

# Project specific imports
from baxter_cashier_manipulation.srv import GetUserPose
from baxter_cashier_manipulation.srv import RecogniseBanknote
import rosgraph.masterapi


class CashierPose:
    def __init__(self, x1, y1, z1, x2, y2, z3, w):
        self.transformation_x = x1
        self.transformation_y = y1
        self.transformation_z = z1

        self.rotation_x = x2
        self.rotation_y = y2
        self.rotation_z = z3
        self.rotation_w = w

    def __str__(self):
        return "{} {} {} {} {} {} {}".format(self.transformation_x,
                                             self.transformation_y,
                                             self.transformation_z,
                                             self.rotation_x,
                                             self.rotation_y,
                                             self.rotation_z,
                                             self.rotation_w)

    def _get_position_and_orientation(self):
        position = Point(self.transformation_x,
                         self.transformation_y,
                         self.transformation_z)

        orientation = Quaternion(self.rotation_x,
                                 self.rotation_y,
                                 self.rotation_z,
                                 self.rotation_w)

        return position, orientation

    def get_pose(self):
        position, orientation = self._get_position_and_orientation()
        return Pose(position=position, orientation=orientation)

    def get_pose_stamped(self):
        pose = self.get_pose()
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        return PoseStamped(header=header, pose=pose)

    def is_empty(self):
        value = [self.transformation_x,
                 self.transformation_y,
                 self.transformation_z,
                 self.rotation_x,
                 self.rotation_y,
                 self.rotation_z,
                 self.rotation_w]

        return all(map(lambda v: v == 0, value))


class BaxterArm():
    def __init__(self, side_name):
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
        """ String representation of the arm, either 'left' or 'right' string
        will be returned. Useful when the string representation of the arm
        is needed, like when accessing the IK solver."""
        return self._side_name

    def is_left(self):
        return True if self._side_name == "left" else False

    def is_right(self):
        return True if self._side_name == "right" else False


class Shopkeeper:
    def __init__(self):
        # This is the camera topic to be used for money recognition (Baxter's
        # head camera or RGB-D camera)
        self._money_recognition_camera_topic = "/camera/rgb/image_rect_color"

        # Baxter's libms configured
        self.left_arm = BaxterArm("left")
        self.right_arm = BaxterArm("right")

        # The static poses to move arm to Baxter's head camera, ideally to
        # use the head camera for money recognition.
        self.pose_to_head_camera_left_hand = CashierPose(0.42974, 0.1787,
                                                         0.74217, 0.49304,
                                                         -0.52719, 0.47738,
                                                         0.50108)

        self.pose_to_head_camera_right_hand = CashierPose(0.41502, -0.15434,
                                                          0.71501, -0.49805,
                                                          0.49115, 0.48121,
                                                          0.52835)

        # TODO: Make this zero, is just for testing purposes set to 3
        self.amount_due = 3

    def interaction(self):
        """
        Handles the main logic of detecting the entrance of new customer, and
        determining if the next action is to get or give money.
        """
        while self.amount_due != 0:
            left_pose, right_pose = self.get_pose_from_space()

            if not pose.is_empty():
                baxter_arm, joint_config = self.ik_solver(left_pose.get_pose_stamped())

                # If the left hand can't reach the pose, try with the right hand
                if baxter_arm is None:
                    baxter_arm, joint_config = self.ik_solver(right_pose.get_pose_stamped())

                if baxter_arm is not None:
                    if self.amount_due < 0:  # Baxter owns money
                        self.give_money_to_customer(baxter_arm, joint_config)
                    else:  # Customer owns money
                        self.take_money_from_customer(baxter_arm, joint_config)

    def take_money_from_customer(self, arm, joints_configurations):
        arm.limb.move_to_joint_positions(joints_configurations)

        # Open/Close the Gripper to catch the money from people's hand
        arm.gripper.open()
        time.sleep(1)

        arm.gripper.close()
        time.sleep(1)

        # Calculate the IK to move the hand to Baxter's head camera
        if arm.is_left():
            pose_stamped = self.pose_to_head_camera_left_hand.get_pose_stamped()
        else:
            pose_stamped = self.pose_to_head_camera_right_hand.get_pose_stamped()

        joints_to_move_to_head = self.ik_solver(pose_stamped, arm)

        if joints_to_move_to_head is not None:
            arm.limb.move_to_joint_positions(joints_to_move_to_head)
            recognised_banknote = self.get_banknote_value()

            time.sleep(5)

            if recognised_banknote is not None:
                print "Received: " + recognised_banknote
                self.amount_due -= int(recognised_banknote)
                # TODO: Put the banknote to the table
            else:
                print "Unable to recognise banknote"
        else:
            print "Wasn't able to move limb to head camera"

    def get_banknote_value(self):
        # This method blocks until the service 'get_user_pose' is available
        rospy.wait_for_service('bank_note_recogniser')

        try:
            # Handle for calling the service
            recognise_banknote = rospy.ServiceProxy('recognise_banknote',
                                                    RecogniseBanknote)

            # Use the handle as any other normal function
            return recognise_banknote(camera_topic=self._money_recognition_camera_topic)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        return None

    def give_money_to_customer(self, arm, joint_configurations):
        if self.amount_due <= -5:
            money_to_give_back = 5
            self.take_a_five_banknote()
        elif self.amount_due >= -4:
            money_to_give_back = 1
            self.take_a_one_banknote()

        arm.limb.move_to_joint_positions(joint_configurations)

        # Waiting user to reach the robot to get the money
        time.sleep(2)
        arm.gripper.open()

        self.amount_due += money_to_give_back

    def set_neutral_position_of_limb(self, arm):
        '''
        Set limb's neutral position.
        '''
        print "Moving limb to neutral position..."
        arm.limb.move_to_neutral()
        print "Limb's neutral position set."

    def ik_solver(self, pose_stamped, arm=self.left_arm):
        '''
        Performs Inverse Kinematic on a given limb and pose.

        Given the limb and a target pose, will calculate the joint
        configuration of the limb.
        '''

        ns = "ExternalTools/" + str(arm) + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()

        ikreq.pose_stamp.append(pose_stamped)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
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
                print "No solution found."

        return None, None

    def get_list_of_users(self):
        master = rosgraph.masterapi.Master('/camera_depth_optical_frame')
        print master.getPublishedTopics('/cob_body_tracker')

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
        left_hand_pose = CashierPose(x1, y1, z1, x2, y2, z2, w)

        x1, y1, z1 = right_hand.transformation
        x2, y2, z2, w = right_hand.rotation
        right_hand_pose = CashierPose(x1, y1, z1, x2, y2, z2, w)

        return left_hand_pose, right_hand_pose


def init():
    print("Initializing node... ")
    rospy.init_node("baxter_cashier")

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()

    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()


if __name__ == '__main__':
    init()
    baxter = Shopkeeper()
    baxter.get_list_of_users()

    while True:
        baxter.interaction()
