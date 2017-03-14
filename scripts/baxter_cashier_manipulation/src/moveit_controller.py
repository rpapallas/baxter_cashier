#!/usr/bin/env python
import sys
import copy
import rospy
import geometry_msgs.msg

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

import moveit_commander
import moveit_msgs.msg

from moveit_commander import MoveGroupCommander
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from std_msgs.msg import Header

from environment_factory import EnvironmentFactory
from baxter_pose import BaxterPose

class MoveItArm:
    def __init__(self, side_name):
        self._side_name = side_name
        self.limb = MoveGroupCommander("{}_arm".format(side_name))

        self.limb.set_goal_position_tolerance(0.01)
        self.limb.set_goal_orientation_tolerance(0.01)

        self.gripper = None

        # TODO: Calibrate gripper

    def __str__(self):
        """ String representation of the arm, either 'left' or 'right' string
        will be returned. Useful when the string representation of the arm
        is needed, like when accessing the IK solver."""
        return self._side_name

    def is_left(self):
        return True if self._side_name == "left" else False

    def is_right(self):
        return True if self._side_name == "right" else False


class MoveItPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()

        self.left_arm = MoveItArm("left")
        self.right_arm = MoveItArm("right")
        self.active_hand = None

        self.scene = moveit_commander.PlanningSceneInterface()
        # rospy.sleep(2)
        self._create_scene()

        #We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=30)
        # moveit_commander.os._exit(0)

    def _create_scene(self):
        # Initialise the Factory of Environments
        EnvironmentFactory.initialize()

        # Get University of Leeds (Robotic's Lab) environment with the obstacles of it.
        environment = EnvironmentFactory.get_robotics_lab_environment()

        # For each obstacle in the environment added to the scene
        for obstacle in environment.get_obstacles():
            obstacle.set_frame_id(self.robot.get_planning_frame())
            self.scene.add_box(obstacle.name, obstacle.pose, obstacle.size)

    def is_pose_reachable_by_robot(self, baxter_pose, arm=None):
        if arm is None:
            arm = self.left_arm

        arm.limb.set_pose_target(baxter_pose.get_pose())
        plan = self.right_arm.limb.plan()

        if plan.joint_trajectory.points == [] and arm is self.left_arm:
            self.is_pose_reachable_by_robot(baxter_pose, arm=self.right_arm)

        return False if plan.joint_trajectory.points == [] else True

    def open_gripper(self):
        pass

    def close_gripper(self):
        pass

    def move_hand_to_head_camera(self):
        pass

    def move_to_position(self, baxter_pose, arm=None):
        if arm is None:
            arm = self.left_arm

        arm.limb.set_pose_target(baxter_pose.get_pose())
        arm.limb.set_planner_id("RRTConnectkConfigDefault")
        plan = arm.limb.plan()

        if plan.joint_trajectory.points == [] and arm is self.left_arm:
            self.move_to_position(baxter_pose, arm=self.right_arm)

        arm.limb.go(wait=True)

    def  set_neutral_position_of_limb(self, arm):
        pass


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    planner = MoveItPlanner()

    pose = BaxterPose(0.72651, -0.041037, 0.19097, 0.56508, -0.5198, -0.54332, -0.33955)
    print planner.is_pose_reachable_by_robot(pose)
    moveit_commander.os._exit(0)
