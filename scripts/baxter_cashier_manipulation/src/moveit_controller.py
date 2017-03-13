class MoveItArm():
    def __init__(self, side_name):
        self._side_name = side_name

        self.left_arm = moveit_commander.MoveGroupCommander(side_name)
        self.gripper = Gripper(side_name.replace("_arm", ""), CHECK_VERSION)

        # Calibrate the gripper
        self.gripper.calibrate()

    def __str__(self):
        """ String representation of the arm, either 'left' or 'right' string
        will be returned. Useful when the string representation of the arm
        is needed, like when accessing the IK solver."""
        return self._side_name

    def is_left(self):
        return True if self._side_name == "left_arm" else False

    def is_right(self):
        return True if self._side_name == "right_arm" else False




class MoveitPlanner:
    def __init__(self):
        self.moveit_commander.roscpp_initialize()

        # This object is an interface to the robot as a whole.
        self.robot = moveit_commander.RobotCommander()

        # This object is an interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        # This object is an interface to one group of joints. In this case the
        # group is the joints in the left arm. This interface can be used to
        # plan and execute motions on the left arm.
        self.left_arm = MoveItArm("left_arm")
        self.right_arm = MoveItArm("right_arm")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        rospy.sleep(10)

    def is_pose_reachable_by_robot(self, baxter_pose):
        baxter_arm, solution = self.ik_solver(baxter_pose.get_pose_stamped())
        return True if solution is not None else False

    def open_gripper(self):
        self.active_hand.gripper.open()
        time.sleep(1)

    def close_gripper(self):
        self.active_hand.gripper.close()
        time.sleep(1)

    def move_hand_to_head_camera(self):
        pass

    def move_to_position(self, baxter_pose):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = baxter_pose.rotation_w
        pose_target.position.x = baxter_pose.transformation_x
        pose_target.position.y = baxter_pose.transformation_y
        pose_target.position.z = baxter_pose.transformation_z

        arm.set_pose_target(pose_target)

        plan1 = arm.plan()
        arm.go(wait=True)
        arm.clear_pose_targets()
