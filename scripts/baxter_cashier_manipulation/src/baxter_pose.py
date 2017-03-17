from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
import rospy


class BaxterPose:
    """Represents a pose that can be used in IK Solvers to solve."""

    def __init__(self, x1, y1, z1, x2, y2, z3, w):
        """ Initialise the class with the given attributes """
        self.transformation_x = x1
        self.transformation_y = y1
        self.transformation_z = z1

        self.rotation_x = x2
        self.rotation_y = y2
        self.rotation_z = z3
        self.rotation_w = w

    def __str__(self):
        """String representation of the pose"""
        return "{} {} {} {} {} {} {}".format(self.transformation_x,
                                             self.transformation_y,
                                             self.transformation_z,
                                             self.rotation_x,
                                             self.rotation_y,
                                             self.rotation_z,
                                             self.rotation_w)

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
        """ Returns a Pose object """
        position, orientation = self._get_position_and_orientation()
        return Pose(position=position, orientation=orientation)

    def get_pose_stamped(self):
        pose = self.get_pose()
        header = Header(stamp=rospy.Time.now(), frame_id='base')

        return PoseStamped(header=header, pose=pose)

    def is_empty(self):
        """Checks if the pose is empty (all attributes zeroes)"""
        value = [self.transformation_x,
                 self.transformation_y,
                 self.transformation_z,
                 self.rotation_x,
                 self.rotation_y,
                 self.rotation_z,
                 self.rotation_w]

        return all(map(lambda v: v == 0, value))
