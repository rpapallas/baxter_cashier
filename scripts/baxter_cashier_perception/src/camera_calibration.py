#!/usr/bin/python

# Python specific imports
import argparse
import sys
import os
from os.path import isfile
from os.path import join
from os.path import expanduser
import argparse

# ROS specific imports
import rospy
import tf

# Other imports
import numpy as np
import cv2


class BasicDatabase:
    """Stores the values from the script to a file for later use."""

    def __init__(self):
        """
        Default constructor to setup the directory to store and load the
        values.
        """

        # Path where script will store the files
        path = expanduser("~") + "/baxter_cashier_calibrator_files/"
        self.file_save_directory = path

        # If directory doesn't exist, then create that directory.
        if not os.path.exists(self.file_save_directory):
            os.makedirs(self.file_save_directory)

    def get_available_files(self):
        """
        Returns a list of available files in the directory. This includes only
        files and not sub-directories. The usage of this function is to show
        to the user a possible number of fils to choose from to load values.
        """
        files = [f for f in os.listdir(self.file_save_directory)
                 if isfile(join(self.file_save_directory, f))]

        return files if len(files) > 0 else None

    def load_values(self, file_name):
        """
        Given a file name, this method will load the values from the file and
        will return xyz and rpy values.
        """
        # Load file
        file_path = join(self.file_save_directory, file_name)

        with open(file_path) as f:
            content = f.readlines()

        # Remove whitespace characters like `\n` at the end of each line
        content = [x.strip() for x in content]

        # Parse xyz and rpy (casting to ints)
        xyz = [int(num) for num in content[0:3]]  # First three values
        rpy = [int(num) for num in content[3:]]   # Last three values

        return xyz, rpy

    def save_values_to_file(self, file_name, xyz, rpy):
        """This method will store the xyz and rpy values to file."""

        full_file_path = join(self.file_save_directory, file_name)
        store_values_file = open(full_file_path, "w")

        x, y, z = xyz
        store_values_file.write(str(x) + "\n")
        store_values_file.write(str(y) + "\n")
        store_values_file.write(str(z) + "\n")

        r, p, y = rpy
        store_values_file.write(str(r) + "\n")
        store_values_file.write(str(p) + "\n")
        store_values_file.write(str(y) + "\n")

        store_values_file.close()


class Calibrator:
    """Calibrator aligns two camera's POV to a single one."""

    def __init__(self, base_topic, target_topic, load_from_file=False):
        """
        Class constructor that do some important initialisation.

        - Creates the required rospy configuration
        - Creates the Database instance to load or store values for this script
        - Set the class' values to default or loads from file.
        - Ask from the user to enter file name to store new values if is
        values are not loaded from file.
        """
        # Basic rospy configuration
        rospy.init_node('camera_calibrator_tf_broadcaster')
        self.rate = rospy.Rate(100.0)

        # Flag indicating if the script will load values from file or not.
        self.load_from_file = load_from_file

        # Basic-flat database to store the values of the script.
        self.database = BasicDatabase()

        # The class' brodcasters
        self.broadcaster = tf.TransformBroadcaster()

        # Default values
        self.quaternion = [0, 0, 0, 0]
        self.xyz_transformed = [0, 0, 0]
        self.xyz = [0.0, 0.0, 0.0]
        self.rpy = [0, 0, 0]
        self.file_name = None

        # Topics that will be used to broadcast tf
        self.base_topic = base_topic
        self.target_topic = target_topic

        # Load values from file
        if self.load_from_file:
            # Ask from the user for a file name from a list of possible files,
            # and then load the values from that file.
            self.file_name = self._get_file_name_from_user()

            if self.file_name is not None:
                self.xyz, self.rpy = self.database.load_values(self.file_name)

            # Since we have load the values from file, we want to calculate the
            # quaternion from these values as well as a small tuning on xyz.
            self.calculate_values()

        # Ask for file name to store the new values for this session
        if self.file_name is None or not self.load_from_file:
            if self.file_name is None:
                print "No files found to load. Starting a new configuration..."

            # Ask for new file name
            message = "Enter a new file name to save configuration: "
            self.file_name = raw_input(message)

        # OpenCV for window
        self.cv2 = cv2

        # Initialise the trackbars (sliders) for the CV window
        self._create_trackbars_for_window()

    def _get_file_name_from_user(self):
        """
        Asks from the user to choose a file from a list of possible files found
        in the directory. The user should enter a number from that list
        starting from zero.
        """
        list_of_files = self.database.get_available_files()

        if list_of_files is None: return

        print "Available files:"

        for i in range(0, len(list_of_files)):
            print "{}. {}".format(i, list_of_files[i])

        file_number = int(raw_input("Enter file number from the list: "))

        if file_number >= 0 and file_number < len(list_of_files):
            return list_of_files[file_number]

        return None

    def _create_trackbars_for_window(self):
        """
        Called only once to initialise and created the OpenCV window with
        the trackbars. The values of trackbars will be set to 0 if not loading
        from file, or will be set to the last used values if loaded from file.
        """
        self.cv2.namedWindow('image')

        # Create trackbars
        x, y, z = [int(v) for v in self.xyz]
        self.cv2.createTrackbar('x', 'image', x, 12000, self._callback)
        self.cv2.createTrackbar('y', 'image', y, 12000, self._callback)
        self.cv2.createTrackbar('z', 'image', z, 12000, self._callback)

        r, p, y = [int(v) for v in self.rpy]
        self.cv2.createTrackbar('Roll',  'image', r, 12600, self._callback)
        self.cv2.createTrackbar('Pitch', 'image', p, 12600, self._callback)
        self.cv2.createTrackbar('Yaw',   'image', y, 12600, self._callback)

    def calculate_values(self):
        """
        When xyz and rpy values are given from the user, some formulas needs to
        be applied to get the xyz corrected and the quaternion value.
        """
        def apply_formula(value):
            return value * np.pi / 1800

        # Perform (1000 - 1) to x, y and z using list comprehension
        self.xyz_transformed = [v / 1000.0 - 6 for v in self.xyz]

        # Using map function we get new values for r, p and y
        # based on static formula computed by `apply_formula` function
        roll, pitch, yaw = map(apply_formula, self.rpy)

        # Using Euler method calculates the quaternion from roll, pitch and yaw
        self.quaternion = tf.transformations.quaternion_from_euler(roll,
                                                                   pitch,
                                                                   yaw)

    def _callback(self, _):
        """
        This callback function is called whenever the trackbars from the OpenCV
        window are changed.
        """
        # Get xyz and rpy current position from the trackbars
        self.xyz = self._extract_xyz_from_trackbars()
        self.rpy = self._extract_rpy_from_trackbars()

        # Calculate the new values based on the new configuration
        self.calculate_values()

        # Auto-save new values to file.
        self.database.save_values_to_file(self.file_name, self.xyz, self.rpy)

    def _extract_xyz_from_trackbars(self):
        """Extracts x, y and z values from CV2 trackbars."""

        x = self.cv2.getTrackbarPos('x', 'image')
        y = self.cv2.getTrackbarPos('y', 'image')
        z = self.cv2.getTrackbarPos('z', 'image')

        return [x, y, z]

    def _extract_rpy_from_trackbars(self):
        """Extracts r, p and y values from CV2 trackbars."""

        r = self.cv2.getTrackbarPos('Roll', 'image')
        p = self.cv2.getTrackbarPos('Pitch', 'image')
        y = self.cv2.getTrackbarPos('Yaw', 'image')

        return [r, p, y]

    def calibrate(self):
        """Method performs the basic operation."""
        print "Start publishing tf..."

        while not rospy.is_shutdown():
            img = np.zeros((300, 512, 3), np.uint8)
            self.cv2.imshow('image', img)
            _ = self.cv2.waitKey(1) & 0xFF

            self.broadcaster.sendTransform(tuple(self.xyz_transformed),
                                           tuple(self.quaternion),
                                           rospy.Time.now(),
                                           self.base_topic,
                                           self.target_topic)

            self.rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", action="store_true",
                        help="Load values from file")

    args = parser.parse_args()

    # Topics to be used to publish the tf.
    base_topic = "camera_link"
    target_topic = "torso"

    # Load values from file
    if args.l:  # l for load
        calibrator = Calibrator(base_topic=base_topic,
                                target_topic=target_topic,
                                load_from_file=True)
    else:
        calibrator = Calibrator(base_topic=base_topic,
                                target_topic=target_topic)

    calibrator.calibrate()
