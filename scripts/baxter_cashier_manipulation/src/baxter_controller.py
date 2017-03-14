import time
import rospy

# Baxter specific imports
import baxter_interface
from baxter_interface import Gripper

from baxter_interface import CHECK_VERSION
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

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


class BaxterPlanner:
    def __init__(self):
        self.left_arm = BaxterArm("left")
        self.right_arm = BaxterArm("right")

        self.active_hand = None

    def is_pose_reachable_by_robot(self, baxter_pose):
        baxter_arm, solution = self.ik_solver(baxter_pose.get_pose_stamped())
        return True if solution is not None else False

    def open_gripper(self):
        self.active_hand.gripper.open()
        time.sleep(1)

    def close_gripper(self):
        self.active_hand.gripper.close()
        time.sleep(1)

    def move_hand_to_head_camera(self):
        left_hand = {'left_w0': 2.64343239272, 'left_w1': -0.846373899716, 'left_w2': -2.14527213186, 'left_e0': -2.40988381777, 'left_e1': 2.12494688642, 'left_s0': 0.956820516444, 'left_s1': -0.369305874683}
        right_hand = {'right_s0': -0.852509822867, 'right_s1': -0.521936963078, 'right_w0': 0.557218521199, 'right_w1': 0.93572828061, 'right_w2': -0.926524395883, 'right_e0': 2.33395176877, 'right_e1': 1.99149055787}

        self.active_hand.limb.move_to_joint_positions(left_hand)

    def move_to_position(self, baxter_pose):
        baxter_arm, solution = self.ik_solver(baxter_pose.get_pose_stamped())

        if solution is not None:
            baxter_arm.limb.move_to_joint_positions(solution)
            self.active_hand = baxter_arm
        else:
            self.active_hand = None

    def set_neutral_position_of_limb(self, arm):
        '''
        Set limb's neutral position.
        '''
        print "Moving limb to neutral position..."
        arm.limb.move_to_neutral()
        print "Limb's neutral position set."

    def ik_solver(self, pose_stamped, arm=None):
        '''
        Performs Inverse Kinematic on a given limb and pose.

        Given the limb and a target pose, will calculate the joint
        configuration of the limb.
        '''
        if arm is None:
            arm = self.left_arm

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
