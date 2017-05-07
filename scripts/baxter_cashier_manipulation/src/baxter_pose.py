#!/usr/bin/env python
"""
    Copyright (C)  2016/2017 The University of Leeds and Rafael Papallas.

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
import time

# ROS-wide imports
import rospy
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
from std_msgs.msg import Header


class BaxterPose:
    """Represents a pose that is used in the entire project."""

    def __init__(self, x1, y1, z1, x2, y2, z3, w):
        """Initialise the class with the given attributes."""
        self.transformation_x = x1
        self.transformation_y = y1
        self.transformation_z = z1

        self.rotation_x = x2
        self.rotation_y = y2
        self.rotation_z = z3
        self.rotation_w = w

        self.created = time.time()

    def __str__(self):
        """String representation of the pose."""
        return "{} {} {} {} {} {} {}".format(self.transformation_x,
                                             self.transformation_y,
                                             self.transformation_z,
                                             self.rotation_x,
                                             self.rotation_y,
                                             self.rotation_z,
                                             self.rotation_w)

    def _get_position_and_orientation(self):
        """Will return the position and orientation of the pose."""
        position = Point(self.transformation_x,
                         self.transformation_y,
                         self.transformation_z)

        orientation = Quaternion(self.rotation_x,
                                 self.rotation_y,
                                 self.rotation_z,
                                 self.rotation_w)

        return position, orientation

    def get_pose(self):
        """Will return a Pose object."""
        position, orientation = self._get_position_and_orientation()
        return Pose(position=position, orientation=orientation)

    def get_pose_stamped(self):
        """Will return a pose stamped object of the pose."""
        pose = self.get_pose()
        header = Header(stamp=rospy.Time.now(), frame_id='base')

        return PoseStamped(header=header, pose=pose)

    def is_empty(self):
        """Will check if the pose is empty (all attributes zeroes)."""
        value = [self.transformation_x,
                 self.transformation_y,
                 self.transformation_z,
                 self.rotation_x,
                 self.rotation_y,
                 self.rotation_z,
                 self.rotation_w]

        return all(map(lambda v: v == 0, value))
