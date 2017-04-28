#!/usr/bin/env python
import rospy
import time
import sys
from moveit_controller import MoveItPlanner

if __name__ == '__main__':
    rospy.init_node("demo")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    planner = MoveItPlanner()
    hand = sys.argv[1]  # Either "left" or "right"

    # Set which hand MoveIt to use (based on the command-line argument)
    active_hand = planner.left_arm if hand == "left" else planner.right_arm

    # This will "say" to MoveIt which hand to use
    planner.active_hand = active_hand

    # Pose to get item
    input("Move {} hand to position and click ENTER (Take State)...".format(hand))
    pose1 = planner.get_end_effector_current_pose()

    # Pose to give item over
    input("Move {} hand to position and click ENTER (Give State)...".format(hand))
    pose2 = planner.get_end_effector_current_pose()

    print "Poses recorded."



    input("Click ENTER to move hand to TAKE position...")
    planner.move_to_position(pose1)

    input("Click ENTER to close the gripper now...")
    planner.close_gripper()



    input("Click ENTER to move hand to GIVE position...")
    planner.move_to_position(pose2)

    input("Click ENTER to open the gripper now...")
    planner.open_gripper()
