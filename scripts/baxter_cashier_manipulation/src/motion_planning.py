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

# Baxter specific imports
import baxter_interface
from baxter_interface import Gripper
import baxter_external_devices
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


class CashierPose:
    def __init__(self, x1, y1, z1, x2, y2, z3, w):
        self.transformation_x = 0.12839
        self.transformation_y = 0
        self.transformation_z = 0.06368

        self.rotation_x = 0.542864
        self.rotation_y = 0.542864
        self.rotation_z = 0.453099
        self.rotation_w = 0.453099

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


class Shopkeeper:
    def __init__(self):
        self.left_limb = baxter_interface.Limb("left")
        self.right_limb = baxter_interface.Limb("right")
        self.left_gripper = Gripper("left")
        self.right_gripper = Gripper("right")

        # This is a static (relative) pose of Baxter's head camera.
        self.relative_head_camera_pose = CashierPose(0.12839,   # Trans X
                                                     0,         # Trans Y
                                                     0.06368,   # Trans Z
                                                     0.542864,  # Rotation X
                                                     0.542864,  # Rotation Y
                                                     0.453099,  # Rotation Z
                                                     0.453099)  # Rotation W

    def get_limb_for_side(self, side):
        return self.left_limb if side == "left" else self.right_limb

    def get_gripper_for_side(self, side):
        return self.left_gripper if side == "left" else self.right_gripper

    def move_limb_to_position(self, limb_side, joints_configurations, is_get):
        '''
        Given the limb to be moved as well as the joint configurations
        which is a dictionary of key-value pair with key being the name
        of the joint and value the configuration of that joint, this
        function will configure all the joints to the given
        configuration
        '''

        if is_get:
            self.take_money_from_customer(limb_side, joints_configurations)
        else:
            self.give_money_to_customer(limb_side, joint_configurations)

    def give_money_to_customer(self, limb_side, joints_configurations):
        limb = self.get_limb_for_side(limb_side)
        gripper = self.get_gripper_for_side(limb_side)

        limb.move_to_joint_positions(joints_configurations)

        # Open/Close the Gripper to catch the money from people's hand
        gripper.open()
        time.sleep(2)
        gripper.close()

        # Calculate the IK to move the hand to Baxter's head camera
        pose_stamped = self.relative_head_camera_pose.get_pose_stamped()
        joints_to_move_to_head = self.inverse_kinematic_solver(limb_size,
                                                               pose_stamped)

        limb.move_to_joint_positions(joints_to_move_to_head)

    def take_money_from_customer(self, limb_side, joint_configurations):
        limb = self.get_limb_for_side(limb_side)
        gripper = self.get_gripper_for_side(limb_side)

        limb.move_to_joint_positions(joints_configurations)

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
        limb = self.get_limb_for_side(limb_side)
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

        if any(resp.isValid):
            print("Valid Joint Solution Found")

            for i in range(len(resp.isValid)):
                if resp.isValid[i]:
                    # Format solution into Limb API-compatible dictionary
                    limb_joints = dict(zip(resp.joints[0].name,
                                           resp.joints[0].position))
                    return limb_joints
        else:
            print "No solution found."

        return None

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
            response = get_user_pose(user_number='1', body_part='left_hand')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        x1, y1, z1 = response.transformation
        x2, y2, z2, w = response.rotation

        return CashierPose(x1, y1, z1, x2, y2, z2, w)


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

    baxter = Shopkeeper()
    pose_stamped = baxter.get_pose_from_space().get_pose_stamped()
    joint_configuration = baxter.inverse_kinematic_solver(args.limb,
                                                          pose_stamped)
    if joint_configurations is not None:
        baxter.move_limb_to_position(limb_side=args.limb,
                                     joints_configurations=joint_configuration,
                                     is_get=True)


if __name__ == '__main__':
    sys.exit(main())
