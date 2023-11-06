import numpy as np
import rospy
import skrobot
import sys
import tf
import tf2_ros
import time
import math
import cv2
import pdb
from cv_bridge import CvBridge


from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from dynamic_tf_publisher.srv import SetDynamicTF
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from numpy import pi


class UpperArmHole():

    def __init__(self,
                 camera,
                 **kwargs):
        
        self.camera = camera
        self.sub_image = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.pub_debug_image_raw = rospy.Publisher(self.camera + "/upper_arm_hole/debug_image_raw", Image, queue_size=10)

        self.subscribe()

        
    def subscribe(self):
        sub_image = rospy.Subscriber(self.camera + "/color/image_rect_color", Image, self.cb)

    
    def cb(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = image.header

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
                
            if(self.sub_image is not None):
                print(self.sub_image.shape)
                print(type(self.sub_image))
                self.debug_image_raw = self.bridge.cv2_to_imgmsg(self.sub_image, "bgr8")
                
                self.pub_debug_image_raw.publish(self.debug_image_raw)
            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":
    rospy.init_node("upper_arm_hole", anonymous=True)
    upper_arm_hole = UpperArmHole("module_0")
    upper_arm_hole.run()
    # upper_arm_hole_subscriber = rospy.Subsciriber("/module_0/color/image_rect_color", sensor/Image, upper_arm_hole.cb)
    
