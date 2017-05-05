#!/usr/bin/env python
import rospy
import time
import sys
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from std_msgs.msg import String
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import EndEffectorState

import tf

class CashierPose:
    def __init__(self, x1, y1, z1, x2, y2, z3, w):
        self.transformation_x = x1
        self.transformation_y = y1
        self.transformation_z = z1

        self.rotation_x = x2
        self.rotation_y = y2
        self.rotation_z = z3
        self.rotation_w = w

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

class BaxterRunner:
    def __init__(self):
        self.right_arm = baxter_interface.Limb("right")
        self.left_arm = baxter_interface.Limb("left")

    def get_pose(self, limb):
        if limb == "right":
            quaternion_pose = self.right_arm.endpoint_pose()
        elif limb == "left":
            quaternion_pose = self.left_arm.endpoint_pose()

        position   = quaternion_pose['position']
        x, y, z = [position.x, position.y, position.z]

        orientation = quaternion_pose['orientation']
        x2, y2, z2, w = [orientation.x,
                         orientation.y,
                         orientation.z,
                         orientation.w]

        return BaxterPose(x, y, z, x2, y2, z2, w)

    def move_to(self, limb, pose):
        solution = self.inverse_kinematic_solver(limb, pose)
        limb.move_to_joint_positions(solution)

    def inverse_kinematic_solver(self, limb_side, pose_stamped):
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

    def move_to(self, limb, joint_configurations):
        limb.move_to_joint_positions(joint_configurations)

    def open_gripper(self, side):
        """Will open the gripper of the active hand."""
        arm = self.right_arm if side == "right" else self.left_arm
        self.arm.open_gripper()

    def close_gripper(self):
        """Will close the gripper of the active hand."""
        arm = self.right_arm if side == "right" else self.left_arm
        self.arm.close_gripper()

if __name__ == '__main__':
    rospy.init_node("demo")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    baxter = BaxterRunner()
    hand = sys.argv[1]  # Either "left" or "right"

    active_hand = baxter.left_arm if hand == "left" else baxter.right_arm

    # Pose to get item
    input("Move {} hand to position and click ENTER (Take State)...".format(hand))
    pose1 = baxter.get_pose(active_hand)

    # Pose to give item over
    input("Move {} hand to position and click ENTER (Give State)...".format(hand))
    pose2 = baxter.get_pose(active_hand)

    print "Poses recorded."



    input("Click ENTER to move hand to TAKE position...")
    baxter.move_to(active_hand, pose1.pose_stamped())

    input("Click ENTER to close the gripper now...")
    baxter.close_gripper(hand)



    input("Click ENTER to move hand to GIVE position...")
    baxter.move_to(active_hand, pose2.pose_stamped())

    input("Click ENTER to open the gripper now...")
    baxter.open_gripper(hand)
