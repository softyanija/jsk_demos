#!/usr/bin/env python3

import numpy as np
import rospy
import sys
import tf
import tf2_ros
import time
import math
import cv2
import pdb
import message_filters
from cv_bridge import CvBridge
from numpy import pi
import argparse
import os

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import RotatedRectStamped
from opencv_apps.msg import RotatedRectArrayStamped
from dynamic_tf_publisher.srv import SetDynamicTF
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from multi_device_view.msg import Ellipse

class Test():

    def __init__(self,
                 camera,
                 debug_mode,
                 **kwargs):
        
        self.camera = camera
        self.recognition_object = "upper_arm_hole"
        self.debug_mode = debug_mode
        self.sub_color_image = None
        self.sub_depth_image = None
        self.sub_mask_image = None
        self.sub_parts_rect = None
        self.background_image = None
        self.gray_img = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.diff_before = None
        self.diff_after = None
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.rects_subscribe()

    def rects_subscribe(self):
        rospy.Subscriber(self.camera + "/" + self.recognition_object +  "/general_contours/rectangles", RotatedRectArrayStamped, self.rects_callback)
            
    def rects_callback(self, rects):
        self.sub_parts_rect = rects

        
    def run(self):
        rate = rospy.Rate(10)

        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass


            if self.sub_parts_rect is not None:
                print(self.sub_parts_rect)

            else:
                rospy.logwarn("not recieve image")

if __name__ == "__main__":

    rospy.init_node("diff_test", anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("module_name", type=str, help="module_name")

    args = parser.parse_args()

    diff_test = Test(args.module_name, "normal")

    diff_test.run()
