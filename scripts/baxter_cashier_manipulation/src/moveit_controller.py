#!/usr/bin/env python
"""
MoveIt! Controller.

This  controller  bridge  MoveIt  with Rviz and Baxter as well as it provide an
interface  to interact  with  Baxter  through Moveit to  the project, and  more
specifically to shopkeeper.py

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

import sys
import rospy

# MoveIt! Specific imports
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander

# Project specific imports
from environment_factory import EnvironmentFactory
from baxter_pose import BaxterPose
from baxter_controller import BaxterArm


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

        # This solver seems to be better for finding solution among obstacles
        self.limb.set_planner_id("RRTConnectkConfigDefault")

        # Error tollerance
        self.limb.set_goal_position_tolerance(0.09)
        self.limb.set_goal_orientation_tolerance(0.09)

        self._baxter_arm = BaxterArm(side_name)
        self.gripper = self._baxter_arm.gripper

        # TODO: Calibrate gripper

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


class MoveItPlanner:
    """Will configure and initialise MoveIt to be used in shopkeeper.py."""

    def __init__(self):
        """Initialise the arms of Baxter and setup environment obstacles."""
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()

        # Configure and setup both Baxter's hands.
        self.left_arm = MoveItArm("left")
        self.right_arm = MoveItArm("right")

        # Active hand is  used to  keep a state of  which hand  has recently be
        # moved  from the  planenr. For  example, a  user send  request to this
        # script to reach a pose, the algorithms in  this script will determine
        # how to move  there but is not the user t hat will specify which hand,
        # the algorithm here will try both and find which one, hence by keeping
        # an active hand, we are able to use  it for other operaitons like open
        # and close of the gripper on that hand.
        self.active_hand = None

        # Setup the environment. This will add obstacles to MoveIt world.
        self.scene = moveit_commander.PlanningSceneInterface()

        # Don't delete this; is required for obstacles to appear in Rviz
        rospy.sleep(1)

        self._create_scene()

        # We  create this  DisplayTrajectory  publisher which is  used below to
        # publish trajectories for RVIZ to visualize.
        self.publisher = rospy.Publisher('/move_group/display_planned_path',
                                         moveit_msgs.msg.DisplayTrajectory,
                                         queue_size=30)

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
            obstacle.set_frame_id(self.robot.get_planning_frame())
            self.scene.add_box(obstacle.name, obstacle.pose, obstacle.size)

    def is_pose_reachable_by_robot(self, baxter_pose):
        """Will return true if baxter_pose is reachable by any Baxter arm."""
        return any([self._is_pose_reachable_by_arm(baxter_pose, arm)
                    for arm in [self.left_arm, self.right_arm]])

    def _is_pose_reachable_by_arm(self, baxter_pose, arm):
        """
        Will find out if the arm can reach the pose.

        Given a pose and Baxter's arm will check if the planner can find a plan
        to move the arm to the pose.

        Will return True if it does, false otherwise.
        """
        arm.limb.set_pose_target(baxter_pose.get_pose())
        plan = arm.limb.plan()

        return False if plan.joint_trajectory.points == [] else True

    def open_gripper(self):
        """Will open Baxter's gripper on his active hand."""
        self.active_hand.gripper.open()

    def close_gripper(self):
        """Will close Baxter's gripper on his active hand."""
        self.active_hand.gripper.close()

    def move_hand_to_head_camera(self):
        """Will move Baxter's active hand to head."""
        if self.active_hand is None:
            return

        # These are  static joint  configurations that lead Baxter's hand to be
        # in a  money  detection  pose (i.e the hand is  locating near Baxter's
        # head camera trying to identify the banknote)
        left_hand = {'left_w0': 2.64343239272,
                     'left_w1': -0.846373899716,
                     'left_w2': -2.14527213186,
                     'left_e0': -2.40988381777,
                     'left_e1': 2.12494688642,
                     'left_s0': 0.956820516444,
                     'left_s1': -0.369305874683}

        right_hand = {'right_s0': -0.852509822867,
                      'right_s1': -0.521936963078,
                      'right_w0': 0.557218521199,
                      'right_w1': 0.93572828061,
                      'right_w2': -0.926524395883,
                      'right_e0': 2.33395176877,
                      'right_e1': 1.99149055787}

        config = left_hand if self.active_hand is self.left_arm else right_hand

        # Move Baxter's hand there.
        self.active_hand.limb.set_joint_value_target(config)
        self.active_hand.limb.plan()
        self.active_hand.limb.go(wait=True)

    def move_to_position(self, baxter_pose):
        """
        Will move Baxter hand to the pose.

        Note that the Baxter's arm that  will be used to move is not specified.
        The  algorithm  will  try  both  and  plan  the  first  one to succeed.
        """
        if self.is_pose_reachable_by_robot(baxter_pose):

            # Simply  saying, loop over  both arms (left and right) and try for
            # each  arm check  if it can reach  `baxter_pose`, if it can append
            # this to the list, and  repeat for every  arm in  the list. At the
            # end we just pick the first one (because of [0])
            arm_to_use = [arm for arm in [self.left_arm, self.right_arm]
                          if self._is_pose_reachable_by_arm(baxter_pose,
                                                            arm)][0]
            self.active_hand = arm_to_use
            self.active_hand.limb.clear_pose_targets()
            self.active_hand.limb.set_pose_target(baxter_pose.get_pose())
            self.active_hand.limb.plan()

            self.active_hand.limb.go(wait=True)

    def set_neutral_position_of_limb(self):
        """Will moves Baxter arm to neutral position."""
        left_config = {'left_w0': -0.231247603774,
                                'left_w1': 1.33724775184,
                                'left_w2': -2.79491299553,
                                 'left_e0': -0.0908883616822,
                                 'left_e1': 1.29813124175,
                                 'left_s0': -0.154548564379,
                                 'left_s1': -1.29391279458}

        right_config = {'right_s0': -0.130004871773,
                                  'right_s1': -1.17464578832,
                                  'right_w0': 0.0901213712883,
                                  'right_w1': 1.17196132194,
                                  'right_w2': -0.0766990393943,
                                  'right_e0': 0.647339892488,
                                  'right_e1': 1.49601476339}

        config = left_config if self.active_hand is self.left_arm else right_config
        self.active_hand.limb.set_joint_value_target(config)
        self.active_hand.limb.plan()
        self.active_hand.limb.go(wait=True)


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    planner = MoveItPlanner()

    pose = BaxterPose(0.72651, -0.041037, 0.19097,
                      0.56508, -0.5198, -0.54332, -0.33955)

    planner.is_pose_reachable_by_robot(pose)

    moveit_commander.os._exit(0)
