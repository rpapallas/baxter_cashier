#!/usr/bin/env python
"""
MoveIt! Controller.

This  controller  bridge  MoveIt  with Rviz and Baxter as well as it provide an
interface  to interact  with  Baxter  through Moveit to  the project, and  more
specifically to cashier.py

    Copyright (C)  2016/2017 The University of Leeds and Rafael Papallas

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

# System-wide imports
import sys
import time

# ROS and Baxter specific imports
import rospy
from baxter_interface import Gripper, Limb
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import(DigitalIOState)

# MoveIt! Specific imports
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander

# Project specific imports
from environment_factory import EnvironmentFactory
from baxter_pose import BaxterPose


class MoveItArm:
    """
    Represents Baxter's Arm in MoveIt! world.

    Represents a hand of  Baxter through MoveIt!  Group. Some reusable code has
    been  packed  together  into  this  class  to make  the code more readable.
    """

    def __init__(self, side_name):
        """Will configure and initialise Baxter's hand."""
        self._side_name = side_name

        # This is the reference to Baxter's arm.
        self.limb = MoveGroupCommander("{}_arm".format(side_name))
        self.limb.set_end_effector_link("{}_gripper".format(side_name))

        # Unfortunetly, MoveIt was not able to make the gripper work, neither
        # did was able to find a very good pose using Forward Kinematics,
        # so instead the Baxter SDK is used here to do these two jobs.
        self.gripper = Gripper(side_name, CHECK_VERSION)
        self.gripper.calibrate()
        self._limb = Limb(side_name)

        # This solver seems to be better for finding solution among obstacles
        self.limb.set_planner_id("RRTConnectkConfigDefault")

        # Error tollerance should be as low as possible for better accuracy.
        self.limb.set_goal_position_tolerance(0.01)
        self.limb.set_goal_orientation_tolerance(0.01)

    def __str__(self):
        """
        String representation of the arm.

        String representation of the arm,  either 'left' or 'right' string will
        be returned.  Useful  when the  string  representation  of the  arm  is
        needed, like when accessing the IK solver.
        """
        return self._side_name

    def is_left(self):
        """Will return True if this is the left arm, false otherwise."""
        return True if self._side_name == "left" else False

    def is_right(self):
        """Will return True if this is the right arm, false otherwise."""
        return True if self._side_name == "right" else False

    def open_gripper(self):
        """Will open Baxter's gripper on his active hand."""
        # Block ensures that the function does not return until the operation
        # is completed.
        self.gripper.open(block=True)

    def close_gripper(self):
        """Will close Baxter's gripper on his active hand."""
        # Block ensures that the function does not return until the operation
        # is completed.
        self.gripper.close(block=True)


class MoveItPlanner:
    """Will configure and initialise MoveIt to be used in cashier.py."""

    def __init__(self):
        """Initialise the arms of Baxter and setup environment obstacles."""
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()

        # Configure and setup both Baxter's hands.
        self.left_arm = MoveItArm("left")
        self.right_arm = MoveItArm("right")

        # Active hand is  used to  keep a state of  which hand  has recently be
        # moved  from the  planner. For  example, a  user send  request to this
        # script to reach a pose, the algorithms in  this script will determine
        # how to move  there but is not the user that will specify which hand,
        # the algorithm here will try both and find which one, hence by keeping
        # an active hand, we are able to use  it for other operaitons like open
        # and close of the gripper on that hand.
        self.active_hand = None

        # Setup the environment. This will add obstacles to MoveIt world.
        self.scene = moveit_commander.PlanningSceneInterface()

        # NOTE: Don't delete this; is required for obstacles to appear in Rviz
        rospy.sleep(1)

        self._create_scene()

        # We  create this  DisplayTrajectory  publisher which is  used below to
        # publish trajectories for RVIZ to visualize.
        self.publisher = rospy.Publisher('/move_group/display_planned_path',
                                         moveit_msgs.msg.DisplayTrajectory,
                                         queue_size=30)

    def release_moveit_from_robot(self, side):
        pub = rospy.Publisher('/robot/digital_io/{}_lower_cuff/state'.format(side), DigitalIOState, queue_size=10)

        timeout_start = time.time()
        timeout = 1   # [seconds]
        while time.time() < timeout_start + timeout:
            pub.publish(1, True)

    def is_pose_within_reachable_area(self, pose):
        """
            Determine if the pose is within the robot's rechable area.

            Given a pose, this method will check if the pose is within the
            robot's reachable area by doing boundary checks. The reachable area
            is exactly above the table.
        """
        upper_x = 1
        lower_x = 0.3

        upper_y = 0.5
        lower_y = -0.7

        upper_z = 0.5
        lower_z = 0

        pose_x = pose.transformation_x
        pose_y = pose.transformation_y
        pose_z = pose.transformation_z

        if pose_z <= upper_z and pose_z >= lower_z:
            if pose_y <= upper_y and pose_y >= lower_y:
                if pose_x <= upper_x and pose_x >= lower_x:
                    return True

        return False

    def _create_scene(self):
        """
        Will setup and add obstacles to MoveIt! world.

        Using  Factory  design  pattern  we  are  able  to  define  here  which
        environment  to  setup. The  factory  allows the  flexibility to define
        multiple environments like Robotics Lab, Long Room  and here we can use
        one of those.
        """
        # Initialise the Factory of Environments
        EnvironmentFactory.initialize()

        # Get  University  of  Leeds  (Robotic's  Lab)   environment  with  the
        # obstacles  of it from the  factory. This  environment object contains
        # details about the obstacles  specifically for this  environment. Note
        # the  environment  is a physical  environment that  contains obstacles
        # like walls and table.
        environment = EnvironmentFactory.get_robotics_lab_environment()

        # For each obstacle in the environment add it to the scene
        for obstacle in environment.get_obstacles():
            # Keep the table obstacle to be used later for pose elimination
            if obstacle.name == "table":
                self.table_obstacle = obstacle

            obstacle.set_frame_id(self.robot.get_planning_frame())
            self.scene.add_box(obstacle.name, obstacle.pose, obstacle.size)

    def move_hand_to_head_camera(self):
        """Will move Baxter's active hand to head."""
        if self.active_hand is None:
            return

        # These are  static joint  configurations that lead Baxter's hand to be
        # in a  money  detection  pose (i.e the hand is  locating near Baxter's
        # head camera trying to identify the banknote)
        left_hand = {'left_w0': -0.384645682562,
                             'left_w1': 0.725956407867,
                             'left_w2': -2.26300515733,
                             'left_e0': -2.39032556272,
                             'left_e1': 2.16636436769,
                             'left_s0': 0.884723419413,
                             'left_s1': -0.324436936638}

        right_hand = {'right_s0': -0.852509822867,
                      'right_s1': -0.521936963078,
                      'right_w0': 0.557218521199,
                      'right_w1': 0.93572828061,
                      'right_w2': -0.926524395883,
                      'right_e0': 2.33395176877,
                      'right_e1': 1.99149055787}

        config = left_hand if self.active_hand.is_left() else right_hand

        # Move Baxter's hand there.
        self.active_hand.limb.set_joint_value_target(config)
        # self.active_hand.limb.plan()
        self.active_hand.limb.go(wait=True)
        self.release_moveit_from_robot(self.active_hand._side_name)

    def move_to_position(self, baxter_pose, arm):
        """Will move Baxter hand to the pose."""
        self.active_hand = arm
        self.active_hand.limb.clear_pose_targets()
        self.active_hand.limb.set_pose_target(baxter_pose.get_pose())
        self.active_hand.limb.go(wait=True)
        self.release_moveit_from_robot(self.active_hand._side_name)

    def leave_banknote_to_the_table(self):
        """
        Will leave the banknote to the table.

        Will move the hand to a pose to depose the banknote to the table,
        by openning the gripper and leaving the banknote to the table.
        """
        # Pose for Baxter's left arm
        pose_left = BaxterPose(0.807502569306,
                               -0.0199779026662,
                               -0.0804409779662,
                               -0.352530183014,
                               0.67035681971,
                               -0.623037729619,
                               -0.195366813466)

        # Pose for Baxter's right arm
        pose_right = BaxterPose(0.876858771261,
                                0.0543512044227,
                                -0.0689541072762,
                                -0.368683595952,
                                0.681493836454,
                                0.435025807256,
                                0.458684100414)

        # Identify which is the active hand and use the configuration
        # accordingly
        pose = pose_left if self.active_hand.is_left() else pose_right

        self.move_to_position(pose, self.active_hand)
        self.open_gripper()
        self.set_neutral_position_of_limb()

    def set_neutral_position_of_limb(self):
        """Will moves Baxter arm to neutral position."""
        left_configuration = {'left_s0': 0.0,
                                           'left_s1': -0.55,
                                           'left_e0': 0.0,
                                           'left_e1': 0.75,
                                           'left_w0': 0.0,
                                           'left_w1': 1.26,
                                           'left_w2': 0.0}

        right_configuration = {'right_s0': 0.0,
                                             'right_s1': -0.55,
                                             'right_e0': 0.0,
                                             'right_e1': 0.75,
                                             'right_w0': 0.0,
                                             'right_w1': 1.26,
                                             'right_w2': 0.0}
        config = left_configuration if self.active_hand.is_left() else right_configuration
        self.active_hand.limb.set_joint_value_target(config)
        # self.active_hand.limb.plan()
        self.active_hand.limb.go(wait=True)
        self.release_moveit_from_robot(self.active_hand._side_name)

    def get_end_effector_current_pose(self, side_name):
        """
        Will return the current pose of the end-effector.

        This method will return the current pose of the given side end-effector
        """
        arm = self.left_arm if side_name == "left" else self.right_arm
        pose = arm._limb.endpoint_pose()

        position = pose["position"]
        x, y, z = [position.x, position.y, position.z]

        orientation = pose["orientation"]
        x2, y2, z2, w = [orientation.x,
                         orientation.y,
                         orientation.z,
                         orientation.w]

        return BaxterPose(x, y, z, x2, y2, z2, w)

    def open_gripper(self):
        """Will open the gripper of the active hand."""
        self.active_hand.open_gripper()

    def close_gripper(self):
        """Will close the gripper of the active hand."""
        self.active_hand.close_gripper()


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    planner = MoveItPlanner()

    # Command to test here

    moveit_commander.os._exit(0)
