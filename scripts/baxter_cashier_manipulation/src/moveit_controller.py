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

if __name__ == '__main__':
    #First initialize moveit_commander and rospy.
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

    #Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    #Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    print "============ Waiting for RVIZ..."
    rospy.sleep(2)

    print "============ Starting tutorial "

    scene = moveit_commander.PlanningSceneInterface()

    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.8
    p.pose.position.y = 0
    p.pose.position.z = -0.54
    scene.add_box("table", p, (1, 1.5, 0.8))

    p2 = PoseStamped()
    p2.header.frame_id = robot.get_planning_frame()
    p2.pose.position.x = 0.80
    p2.pose.position.y = 0
    p2.pose.position.z = -0.40
    scene.add_box("table_obj", p2, (0.5, 0.2, 0.8))

    #Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
    group_both_arms = MoveGroupCommander("both_arms")
    group_both_arms.set_goal_position_tolerance(0.01)
    group_both_arms.set_goal_orientation_tolerance(0.01)

    group_left_arm = MoveGroupCommander("left_arm")
    group_left_arm.set_goal_position_tolerance(0.01)
    group_left_arm.set_goal_orientation_tolerance(0.01)

    group_right_arm = MoveGroupCommander("right_arm")
    group_right_arm.set_goal_position_tolerance(0.01)
    group_right_arm.set_goal_orientation_tolerance(0.01)
#
    #We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

    #Call the planner to compute the plan and visualize it if successful.
    print "============ Generating plan for left arm"
    pose_target_left = geometry_msgs.msg.Pose()
    pose_target_left.orientation.x = 0.73567
    pose_target_left.orientation.y = -0.17577
    pose_target_left.orientation.z = 0.22726
    pose_target_left.orientation.w = 0.67253
    pose_target_left.position.x = 0.81576
    pose_target_left.position.y = 0.093893
    pose_target_left.position.z = 0.2496
    group_left_arm.set_pose_target(pose_target_left)

    #group_left_arm.set_position_target([0.75,0.27,0.35])
    group_left_arm.set_planner_id("RRTConnectkConfigDefault")
    plan_1eft = group_left_arm.plan()
    #print "Trajectory time (nsec): ", plan_left.joint_trajectory.points[len(plan_left.joint_trajectory.points)-1].time_from_start

    rospy.sleep(5)
    print "============ Generating plan for right arm"
    pose_target_right = geometry_msgs.msg.Pose()
    pose_target_right.orientation.x = 0.56508
    pose_target_right.orientation.y = -0.5198
    pose_target_right.orientation.z = -0.54332
    pose_target_right.orientation.w = -0.33955
    pose_target_right.position.x = 0.72651
    pose_target_right.position.y = -0.041037
    pose_target_right.position.z = 0.19097
    group_right_arm.set_pose_target(pose_target_right)

    #group_right_arm.set_position_target([0.75,-0.27,0.35])
    plan_right = group_right_arm.plan()
    #print "Trajectory time (nsec): ", plan_right.joint_trajectory.points[len(plan_right.joint_trajectory.points)-1].time_from_start
    rospy.sleep(5)


    print "============ Generating plan for both arms"
    #group_both_arms.set_pose_target(pose_target_right, pose_target_left)
    group_both_arms.set_pose_target(pose_target_right, 'right_gripper')
    group_both_arms.set_pose_target(pose_target_left, 'left_gripper')

    plan_both = group_both_arms.plan()
    print "Trajectory time (nsec): " + str(plan_both.joint_trajectory.points[len(plan_both.joint_trajectory.points)-1].time_from_start)
    moveit_commander.os._exit(0)
