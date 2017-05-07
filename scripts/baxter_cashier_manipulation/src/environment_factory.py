#!/usr/bin/env python

"""
Factory for Environments.

This file contains some  static classes that  represents  environments  in real
life. If Baxter for  example is  placed somewhere  in a real  environment let's
name it "Robotics Lab" then we wish to define  obstacles around  Baxter in this
specific  environment.  In  this  class  we  achieve  exactly  this,  for  each
environment that  Baxter can  be, we  define the obstacles around him and using
the  Factory  Pattern   and  Template  design  pattern  we  are  able  to  have
extensibility with a very nice way.

If you need to define a new environment here are the steps:

1. Define a  similar  class with the one listed below: `RoboticsLabEnvironment`
   but make sure the  obstacles  implemented in  `RoboticsLabEnvironment` match
   you own obstacles in  your  environment, and  make sure  you give a sensible
   name for the class.
2. In `EnvironmentFactory`  class,  define a top-level attribute with  the name
   of your new class (see the  one already there: `__robotics_lab_environment`)
3. Implement your getter, as like `def get_robotics_lab_environment():` and use
   similar logic to return your new class back.
4. In `moveit_controller.py` find the line
   `EnvironmentFactory.get_robotics_lab_environment()` and change it to match
   your new getter method.

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

import copy
from geometry_msgs.msg import PoseStamped


class EnvironmentFactory:
    """
    Environment Factory implementing the design pattern.

    In here are defined the getters for the different environments and is the
    class used in other scripts to generate the class environments required.
    """

    _robotics_lab_environment = None

    @staticmethod
    def initialize():
        """Initialise each environment."""
        EnvironmentFactory._robotics_lab_environment = RoboticsLabEnvironment()

    @staticmethod
    def get_robotics_lab_environment():
        """Will return the robotics lab environment."""
        return EnvironmentFactory._robotics_lab_environment.clone()


class Obstacle:
    """This represent an obstacle in real world."""

    def __init__(self, obstalce_name, x, y, z, shape_size):
        """
        Will configure the obstacle details and set it's attributes.

        - obstalce_name: is the name of the obstacle.
        - x, y and z: is the position or pose of the obstacle in the world.
        - shape_size: is a triple tuple with height, width and depth of the
        object or obstacle.
        """
        self.name = obstalce_name

        # The pose of where the obstacle is
        self.pose = PoseStamped()
        self.pose.pose.position.x = x
        self.pose.pose.position.y = y
        self.pose.pose.position.z = z

        # Pose Header Frame ID is None because it needs to be set for the
        # specific scene, which is not available at the time the obstacle
        # is created.
        self.pose.header.frame_id = None

        # This is a triple tuple (h, w, z) representing the size of the
        # obstacle
        self.size = shape_size

    def set_frame_id(self, id):
        """
        Will set the pose's header frame ID.

        It is important, for  the  obstacle to appear in the MoveIt Rviz to set
        this to  `robot.get_planning_frame()`,  since we  don't  have this info
        in here,  we need  to set  this later.  Make sure  you  have  set  this
        otherwise you will not be able to visualise the obstacle in Rviz.
        """
        self.pose.header.frame_id = id


class Environment:
    """This is the template class of the Template design pattern."""

    # Obstacles represents a list of obstacles
    _obstacles = None

    def clone(self):
        """
        Clone itself.

        Required  method  to  clone itself  when Factory  is used  to  get the
        instance.
        """
        pass

    def get_obstacles(self):
        """Will return the list with obstacles."""
        return self._obstacles


class RoboticsLabEnvironment(Environment):
    """
    This class represent's University of Leeds, Robotic's Laboratory.

    The obstacles defiend here are specifically to that environment. This is
    a subclass of the environment template of the Template design pattern.
    """

    def __init__(self):
        """
        Default constructor.

        Will initialise the obstacles attribute to empty list and will call the
        method to create the obstacles.
        """
        self._obstacles = []
        self._create_obstalces()

    def _create_obstalces(self):
        """
        Generate and append the obstacles to the class.

        In here are the obstacles relevant to this specific environment.
        """
        side_wall = Obstacle(obstalce_name="side_wall",
                                           x=0.6,
                                           y=1,
                                           z=0,
                                           shape_size=(4, 0.2, 3))

        self._obstacles.append(side_wall)

        back_wall = Obstacle(obstalce_name="back_wall",
                                           x=-1,
                                           y=0,
                                           z=0,
                                           shape_size=(0.2, 4, 3))

        self._obstacles.append(back_wall)

        table = Obstacle(obstalce_name="table",
                                       x=0.7,
                                       y=-0.1,
                                       z=-0.53,
                                       shape_size=(0.8, 1.2, 0.7))

        self._obstacles.append(table)

        camera_tripod = Obstacle(obstalce_name="camera_tripod",
                                               x=0.6,
                                               y=-1.2,
                                               z=-0.54,
                                               shape_size=(1, 0.3, 1.8))

        self._obstacles.append(camera_tripod)
        # width, length, height

    def clone(self):
        """Required method for the Template design pattern."""
        return copy.copy(self)
