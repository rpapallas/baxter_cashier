#!/usr/bin/env python
import rospy
from baxter_interface import Gripper, Limb
from baxter_interface import CHECK_VERSION

# MoveIt! Specific imports
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander

rospy.init_node('move_group_python_interface', anonymous=True)

# MoveIt!
limb = MoveGroupCommander("left_arm")
limb.set_end_effector_link("left_gripper")
limb.set_planner_id("RRTConnectkConfigDefault")
limb.set_goal_position_tolerance(0.01)
limb.set_goal_orientation_tolerance(0.01)
scene = moveit_commander.PlanningSceneInterface()
publisher = rospy.Publisher('/move_group/display_planned_path',
                            moveit_msgs.msg.DisplayTrajectory,
                            queue_size=30)

# Baxter's default planner
_limb = Limb("left")
gripper = Gripper("left", CHECK_VERSION)
gripper.open(block=True)
gripper.close(block=True)

# Move left hand to head camera - MOVEIT
left_hand = {'left_w0': 2.64343239272,
             'left_w1': -0.846373899716,
             'left_w2': -2.14527213186,
             'left_e0': -2.40988381777,
             'left_e1': 2.12494688642,
             'left_s0': 0.956820516444,
             'left_s1': -0.369305874683}
limb.set_joint_value_target(left_hand)
limb.go(wait=True)

# Move to neutral position - BAXTER'S DEFAULT
_limb.move_to_neutral()

moveit_commander.os._exit(0)
