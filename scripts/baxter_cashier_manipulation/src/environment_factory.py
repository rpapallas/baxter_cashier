import copy
from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

class Obstacle:
    def __init__(self, obstalce_name, x, y, z, shape_size):
            self.name = obstalce_name

            self.pose = PoseStamped()
            self.pose.header.frame_id = None
            self.pose.pose.position.x = x
            self.pose.pose.position.y = y
            self.pose.pose.position.z = z

            self.size = shape_size

    def set_frame_id(self, id):
        self.pose.header.frame_id = id


class Environment:
    _obstacles = None

    def clone(self):
        pass

    def get_obstacles(self):
        return self._obstacles


class RoboticsLabEnvironment(Environment):
    def __init__(self):
        self._obstacles = []
        self._create_obstalces()

    def _create_obstalces(self):
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
                                   x=0.8,
                                   y=0,
                                   z=-0.54,
                                   shape_size=(1, 1.5, 0.8))

        self._obstacles.append(table)

        camera_tripod = Obstacle(obstalce_name="camera_tripod",
                                                   x=0.8,
                                                   y=-1,
                                                   z=-0.54,
                                                   shape_size=(1, 0.3, 1.8))

        self._obstacles.append(camera_tripod)

    def clone(self):
        return copy.copy(self)

class EnvironmentFactory:
    __robotics_lab_environment = None

    @staticmethod
    def initialize():
        EnvironmentFactory.__robotics_lab_environment = RoboticsLabEnvironment()

    @staticmethod
    def get_robotics_lab_environment():
        return EnvironmentFactory.__robotics_lab_environment.clone()
