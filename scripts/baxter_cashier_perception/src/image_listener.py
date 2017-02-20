#!/usr/bin/env python

"""
Based: http://answers.ros.org/question/210294/ros-python-save-snapshot-from-camera/
"""

import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageListener:
    def __init__(self, image_topic):
        # self.image_pub = rospy.Publisher(image_topic, Image)
        self.bridge = CvBridge()
        time.sleep(10)
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e
        else:
            self.image_sub.unregister()

if __name__ == '__main__':
    ic = ImageListener("/camera/rgb/image_raw")
    rospy.init_node('image_listener', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"

    cv2.destroyAllWindows()
