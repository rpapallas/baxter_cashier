#!/usr/bin/python

"""
This class allows two cameras to align with each other by publishing a tf.
Creator: Muhannad, University of Leeds
"""

import rospy
import tf
import numpy as np
import cv2
import os
from os import listdir
from os.path import isfile
from os.path import join
from os.path import expanduser
import argparse
import signal
import sys

class Calibrator:
    """Calibrator aligns two camera's POV to a single one."""

    def __init__(self, base_topic, target_topic, load_from_file=False):
        """Default constructor configuring the class properties."""
        self.file_save_directory = expanduser("~") + "/baxter_cashier_calibrator_files/"
        rospy.init_node('camera_calibrator_tf_broadcaster')
        self.rate = rospy.Rate(100.0)

        # Some initialisation
        self.load_from_file = load_from_file

        if self.load_from_file:
            self.load_values_from_file()
        else:
            self.quaternion = [0, 0, 0, 0]
            self.xyz = [0.0, 0.0, 0.0]
            self.rpy = [0, 0, 0]

        self.xyz_transformed = [0, 0, 0]

        # The class' brodcasters
        self.brodcaster_1 = tf.TransformBroadcaster()

        # Topics that will be used to broadcast tf
        self.base_topic = base_topic
        self.target_topic = target_topic

        if not self.load_from_file:
            self.file_name = raw_input("Enter a name for the file to save the values from this session (something so that you can recognise it next time, please do use .txt or anything at the end): ")

        self.cv2 = cv2

        # Initialise the trackbars (sliders) for the CV window
        self._create_trackbars_for_window()

    def load_values_from_file(self):
        # List all files in the store directory
        if not os.path.exists(self.file_save_directory):
            print "Nothing saved so far. Can't load anything."
            self.quaternion = [0, 0, 0, 0]
            self.xyz = [0.0, 0.0, 0.0]
            self.rpy = [0, 0, 0]
            return

        files = [f for f in listdir(self.file_save_directory) if isfile(join(self.file_save_directory, f))]

        if len(files) == 0:
            print "Nothing saved so far. Can't load anything"
            self.quaternion = [0, 0, 0, 0]
            self.xyz = [0.0, 0.0, 0.0]
            self.rpy = [0, 0, 0]
            return

        print "Here are the available files:"

        for i in range(0, len(files)):
            print "{}. {}".format(i, files[i])

        print ""

        file_number = int(raw_input("Please enter the number of the file you wish to load: "))

        if file_number < 0 or file_number >= len(files):
            print "The number you have entered is not in the list. Nothing loaded."
            return

        self.file_name = join(self.file_save_directory, files[file_number])
        with open(self.file_name) as f:
            content = f.readlines()

        # you may also want to remove whitespace characters like `\n` at the end of each line
        content = [x.strip() for x in content]

        x, y, z = [float(num) for num in content[1:4]]
        q1, q2, q3, q4 = [float(num) for num in content[5:9]]
        r, p, y = [float(num) for num in content[10:]]

        self.quaternion = [q1, q2, q3, q4]
        self.xyz = [x, y, z]
        self.rpy = [r, p, y]

        self.calculate_values()

        print "Quaternion and XYZ values set from file successfuly."

    def save_values_to_file(self):
        if not os.path.exists(self.file_save_directory):
            os.makedirs(self.file_save_directory)

        file_name = self.file_name.replace(" ", "_")

        full_file_path = join(self.file_save_directory, file_name)
        store_values_file = open(full_file_path, "w")

        x, y, z = self.xyz
        store_values_file.write("XYZ:" + "\n")
        store_values_file.write(str(x) + "\n")
        store_values_file.write(str(y) + "\n")
        store_values_file.write(str(z) + "\n")

        q1, q2, q3, q4 = self.quaternion
        store_values_file.write("Quaternion:" + "\n")
        store_values_file.write(str(q1) + "\n")
        store_values_file.write(str(q2) + "\n")
        store_values_file.write(str(q3) + "\n")
        store_values_file.write(str(q4) + "\n")

        r, p, y = self.rpy
        store_values_file.write("rpy:" + "\n")
        store_values_file.write(str(r) + "\n")
        store_values_file.write(str(p) + "\n")
        store_values_file.write(str(y) + "\n")

        store_values_file.close()
        print "Values saved to file."


    def _create_trackbars_for_window(self):
        self.cv2.namedWindow('image')

        # Create trackbars
        self.cv2.createTrackbar('x', 'image', int(self.xyz[0]), 3000, self._callback)
        self.cv2.createTrackbar('y', 'image', int(self.xyz[1]), 3000, self._callback)
        self.cv2.createTrackbar('z', 'image', int(self.xyz[2]), 3000, self._callback)

        self.cv2.createTrackbar('Roll',  'image', int(self.rpy[0]), 3600, self._callback)
        self.cv2.createTrackbar('Pitch', 'image', int(self.rpy[1]), 3600, self._callback)
        self.cv2.createTrackbar('Yaw',   'image', int(self.rpy[2]), 3600, self._callback)

    def calculate_values(self):
        def apply_formula(value):
            return value * np.pi / 1800

        self.xyz_transformed = [v / 1000.0 - 1 for v in self.xyz]

        # Using List Comprehension we get new values for r, p and y
        # based on static formula computed by `_get_new_value_for` static
        # method
        roll, pitch, yaw = map(apply_formula, self.rpy)

        # Using Euler method calculates the quaternion from roll, pitch and yaw
        self.quaternion = tf.transformations.quaternion_from_euler(roll,
                                                                   pitch,
                                                                   yaw)
    def _callback(self, _):
        # Tune xyz and rpy with the current positions of the trackbars
        self.xyz = self._extract_xyz_from_trackbars()
        self.rpy = self._extract_rpy_from_trackbars()

        self.calculate_values()

        self.save_values_to_file()
        self._print_current_configuration()

    def _extract_xyz_from_trackbars(self):
        """ Extracts x, y and z values from CV2 trackbars. """

        x = self.cv2.getTrackbarPos('x', 'image')
        y = self.cv2.getTrackbarPos('y', 'image')
        z = self.cv2.getTrackbarPos('z', 'image')

        return [x, y, z]

    def _extract_rpy_from_trackbars(self):
        """ Extracts r, p and y values from CV2 trackbars. """

        r = self.cv2.getTrackbarPos('Roll', 'image')
        p = self.cv2.getTrackbarPos('Pitch', 'image')
        y = self.cv2.getTrackbarPos('Yaw', 'image')

        return [r, p, y]

    def _print_current_configuration(self):
        print "xyz: {} | rpy: {} | q: {}".format(self.xyz,
                                                 self.rpy,
                                                 self.quaternion)

    def calibrate(self):
        """Method performs the basic operation."""
        print "Start publishing to tf..."

        while not rospy.is_shutdown():
            img = np.zeros((300, 512, 3), np.uint8)
            self.cv2.imshow('image', img)
            _ = self.cv2.waitKey(1) & 0xFF

            self.brodcaster_1.sendTransform(tuple(self.xyz_transformed),
                                            tuple(self.quaternion),
                                            rospy.Time.now(),
                                            self.base_topic,
                                            self.target_topic)

            self.rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-l", action="store_true", help="Load XYZ and Quaternion values from file")

    args = parser.parse_args()

    if args.l:
        calibrator = Calibrator(base_topic="camera_depth_optical_frame",
                                target_topic="torso", load_from_file=True)
    else:
        calibrator = Calibrator(base_topic="camera_depth_optical_frame",
                                target_topic="torso")

    calibrator.calibrate()
