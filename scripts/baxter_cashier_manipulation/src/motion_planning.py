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
from baxter_cashier_perception.srv import GetUserPose


class Shopkeeper:
    def __init__(self):
        self.left_limb = baxter_interface.Limb("left")
        self.right_limb = baxter_interface.Limb("right")

    def get_limb_for_side(self, side):
        return self.left_limb if side == "left" else self.right_limb

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
        limb.move_to_joint_positions(joints_configurations)
        # TODO: Open End-Effector
        time.sleep(2)
        # TODO: Close End-Effector
        # TODO: Bring hand to Robot's head

    def take_money_from_customer(self, limb_side, joint_configurations):
        limb = self.get_limb_for_side(limb_side)
        limb.move_to_joint_positions(joints_configurations)

        # Waiting user to reach the robot to get the money
        time.sleep(2)

        # TODO: Ensure that human's hand is touching the money note
        # TODO: Open End-Effector
        # TODO: Return to natural position

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
        self.set_neutral_position_of_limb(limb)

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

    def get_pose_stamped_from_space(self):
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

        x, y, z = response.transformation
        position = Point(x, y, z)

        x, y, z, w = response.rotation
        orientation = Quaternion(x, y, z, w)

        pose = Pose(position=position, orientation=orientation)

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        return PoseStamped(header=hdr, pose=pose)


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
    pose_stamped = baxter.get_pose_stamped_from_space()
    joint_configurations = baxter.inverse_kinematic_solver(args.limb,
                                                           pose_stamped)
    if joint_configurations is not None:
        baxter.move_limb_to_position(args.limb, joint_configurations)


if __name__ == '__main__':
    sys.exit(main())
