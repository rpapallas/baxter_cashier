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


class Shopkeeper:
    def __init__(self):
        # This is the camera topic to be used for money recognition (Baxter's
        # head camera or RGB-D camera)
        self._money_recognition_camera_topic = "/camera/rgb/image_rect_color"

        # Baxter's libms configured
        self.left_limb = baxter_interface.Limb("left")
        self.right_limb = baxter_interface.Limb("right")

        self.left_limb.set_joint_position_speed(0.1)
        self.right_limb.set_joint_position_speed(0.1)

        # Baxter Grippers configured
        self.left_gripper = Gripper("left", CHECK_VERSION)
        self.right_gripper = Gripper("right", CHECK_VERSION)

        self.left_gripper.calibrate()
        self.right_gripper.calibrate()

        # This is a static (relative) pose of Baxter's head camera.
        self.relative_head_camera_pose = CashierPose(0.57354,    # Trans X
                                                     -0.021107,  # Trans Y
                                                     0.74713,    # Trans Z
                                                     -0.02182,   # Rotation X
                                                     -0.55414,   # Rotation Y
                                                     0.77549,    # Rotation Z
                                                     -0.30178)   # Rotation W

    def get_limb_for_side(self, side):
        return self.left_limb if side == "left" else self.right_limb

    def get_gripper_for_side(self, side):
        return self.left_gripper if side == "left" else self.right_gripper

    def move_limb_to_position(self, limb_side, joint_configurations, is_get):
        '''
        Given the limb to be moved as well as the joint configurations
        which is a dictionary of key-value pair with key being the name
        of the joint and value the configuration of that joint, this
        function will configure all the joints to the given
        configuration
        '''

        if is_get:
            self.take_money_from_customer(limb_side, joint_configurations)
        else:
            self.give_money_to_customer(limb_side, joint_configurations)

    def take_money_from_customer(self, limb_side, joints_configurations):
        limb = self.get_limb_for_side(limb_side)
        gripper = self.get_gripper_for_side(limb_side)

        limb.move_to_joint_positions(joints_configurations)

        # Open/Close the Gripper to catch the money from people's hand
        gripper.open()
        time.sleep(2)
        gripper.close()

        return

        # Calculate the IK to move the hand to Baxter's head camera
        pose_stamped = self.relative_head_camera_pose.get_pose_stamped()
        joints_to_move_to_head = self.inverse_kinematic_solver(limb_side,
                                                               pose_stamped)

        if joints_to_move_to_head is not None:
            limb.move_to_joint_positions(joints_to_move_to_head)
            recognised_banknote = None #get_banknote_value()

            if recognised_banknote is not None:
                print "Received: " + recognised_banknote
            else:
                print "Unable to recognise banknote"
                # TODO: Return note back to the user
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

    def give_money_to_customer(self, limb_side, joint_configurations):
        limb = self.get_limb_for_side(limb_side)
        gripper = self.get_gripper_for_side(limb_side)

        limb.move_to_joint_positions(joint_configurations)

        # Waiting user to reach the robot to get the money
        time.sleep(2)

        # TODO: Ensure that human's hand is touching the money note
        gripper.open()
        limb.move_to_neutral()

    def set_neutral_position_of_limb(self, limb):
        '''
        Set limb's neutral position.
        '''
        print "Moving limb to neutral position..."
        limb.move_to_neutral()
        print "Limb's neutral position set."

    def inverse_kinematic_solver(self, limb_side, pose_stamped):
        '''
        Performs Inverse Kinematic on a given limb and pose.

        Given the limb and a target pose, will calculate the joint
        configuration of the limb.
        '''
        # limb = self.get_limb_for_side(limb_side)
        # self.set_neutral_position_of_limb(limb)

        ns = "ExternalTools/" + limb_side + "/PositionKinematicsNode/IKService"
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
            return limb_joints
        else:
            print "No solution found."

        return None

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
            response = get_user_pose(user_number=1, body_part='left_hand')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        x1, y1, z1 = response.transformation
        x2, y2, z2, w = response.rotation

        x = CashierPose(x1, y1, z1, x2, y2, z2, w)
        print x
        return x


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


def setup_args():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    init()
    args = setup_args()
    found = False

    baxter = Shopkeeper()
    baxter.get_list_of_users()

    while True:
        pose = baxter.get_pose_from_space()

        if not pose_stamped.is_empty
            joint_configuration = baxter.inverse_kinematic_solver(args.limb,
                                                                  pose.get_pose_stamped())

            if joint_configuration is not None:
                print joint_configuration
                found = True
                baxter.move_limb_to_position(limb_side=args.limb,
                                             joint_configurations=joint_configuration,
                                             is_get=True)
                baxter.set_neutral_position_of_limb(baxter.get_limb_for_side(args.limb))


if __name__ == '__main__':
    sys.exit(main())




# Valid Joint Solution Found
# {'left_w0': 2.106294876484381, 'left_w1': -1.1005751521035783, 'left_w2': 1.4644689203366548, 'left_e0': -1.6411138752279184, 'left_e1': 2.2642346774524866, 'left_s0': -0.1384362423002121, 'left_s1': 0.6967474257859355}
# I came here
# Moving limb to neutral position...
# Limb's neutral position set.
#
# Exiting example...
# [baxter - http://10.0.0.101:11311] rafael@csros03:~/ros_ws$ rosrun baxter_cashier_manipulation motion_planning.py -l left
# Initializing node...
# Getting robot state...
# Enabling robot...
# [INFO] [WallTime: 1488902771.318287] Robot Enabled
# Moving limb to neutral position...
# Limb's neutral position set.
# 0.422272155762 0.0783126983643 0.708687805176 0.0 0.0 0.0 1.0
# Valid Joint Solution Found
# {'left_w0': 1.8141469376672572, 'left_w1': -1.47079632679, 'left_w2': 1.6268416838401842, 'left_e0': -1.4891417570892072, 'left_e1': 2.2596985265257055, 'left_s0': -0.1514685648331161, 'left_s1': 0.254126947499098}
# I came here
# Moving limb to neutral position...
# Limb's neutral position set.
