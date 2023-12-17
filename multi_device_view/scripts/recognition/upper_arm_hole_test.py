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

class UpperArmHole():

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
        self.sub_morphology_image = None
        self.background_image = None
        self.gray_img = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.pub_debug_image_raw = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "debug_image_raw"), Image, queue_size=10)
        self.pub_debug_depth = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "debug_depth"), Image, queue_size=10)
        self.pub_backround_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "background_image"), Image, queue_size=10)
        self.pub_diff_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "diff_image"), Image, queue_size=10)
        
        self.pub_threshold_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "threshold_image"), Image, queue_size=10)
        self.pub_ellipse_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "ellipse_image"), Image, queue_size=10)
        self.pub_background_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "background_image"), Image, queue_size=10)
        self.pub_diff_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "diff_image"), Image, queue_size=10)
        self.pub_roi_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "roi_image"), Image, queue_size=10)
        self.pub_masked_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "masked_image"), Image, queue_size=10)
        self.pub_rect_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "rect_image"), Image, queue_size=10)
        self.pub_upper_arm_hole_result = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "guide_point"), RotatedRectStamped, queue_size=10)

        self.service_name = "/" + camera + "/set_background"
        self.service = rospy.Service(self.service_name, Empty, self.set_backgound_image)

        self.multi_subscribe()
        self.rects_subscribe()
        self.morphology_subscribe()


    def multi_subscribe(self):
        sub_color_image = message_filters.Subscriber(self.camera + "/color/image_rect_color", Image)
        sub_depth_image = message_filters.Subscriber(self.camera + "/depth/image_rect_raw", Image)
        sub_mask_image = message_filters.Subscriber(self.camera + "/" + self.recognition_object + "_pre" + "/multiply_mask_image/output", Image)
        # pdb.set_trace()

        self.subs = [sub_color_image, sub_depth_image, sub_mask_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.multi_callback)

    def multi_callback(self, color, depth, mask):
        self.sub_color_image = self.bridge.imgmsg_to_cv2(color, "bgr8")
        self.sub_depth_image = self.bridge.imgmsg_to_cv2(depth, "16UC1")
        self.sub_mask_image = self.bridge.imgmsg_to_cv2(mask, "8UC1")
        
        self.header = color.header


    def rects_subscribe(self):
        rospy.Subscriber(self.camera + "/" + self.recognition_object +  "/general_contours/rectangles", RotatedRectArrayStamped, self.rects_callback)

    def rects_callback(self, rects):
        self.sub_parts_rect = rects.rects

    def morphology_subscribe(self):
        rospy.Subscriber(self.camera + "/" + self.recognition_object +  "/morphology/image", Image, self.morphology_callback)
        
    def morphology_callback(self, image):
        self.sub_morphology_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(image, "bgr8"), cv2.COLOR_BGR2GRAY)



    def set_backgound_image(self, req):
        rospy.loginfo("set background image")
        # self.background_image = self.sub_depth_image.copy()
        self.background_image = self.sub_mask_image.copy()
        self.background_is_set = True
        return EmptyResponse()

        
    def run(self):
        rate = rospy.Rate(10)

        rospy.loginfo("Waiting for service call...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo("Service is now available")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if self.sub_color_image is not None:
                # pdb.set_trace()
                self.pub_debug_depth.publish(self.bridge.cv2_to_imgmsg(self.sub_depth_image))
                
                if self.background_is_set:

                    # diff = cv2.absdiff(self.background_image, self.sub_depth_image)
                    diff = cv2.absdiff(self.background_image, self.sub_mask_image)
                    diff_msg = self.bridge.cv2_to_imgmsg(diff, "mono8")
                    diff_msg.header = self.header
                    self.pub_diff_image.publish(diff_msg)
                    self.pub_backround_image.publish(self.bridge.cv2_to_imgmsg(self.background_image))
                    

                    if self.sub_parts_rect is not None:

                        rect_image_buf = self.sub_color_image.copy()
                        
#                        contours, _ = cv2.findContours(diff, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        contours, _ = cv2.findContours(self.sub_morphology_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        # pdb.set_trace()
                        
                        for cnt in contours:
                            rect = cv2.minAreaRect(cnt)
                            box = cv2.boxPoints(rect)
                            box = np.intp(box)
                            is_in_area = False
                            
                            if (rect[0][1] < 150) and (rect[1][0] * rect[1][1] > 300):
                                is_in_area = True

                            if is_in_area == 0:
                                rect_image_buf = cv2.drawContours(rect_image_buf,[box],0,(255,0,0),2)
                            else:
                                rect_image_buf = cv2.drawContours(rect_image_buf,[box],0,(0,0,255),2)

                        self.pub_rect_image.publish(self.bridge.cv2_to_imgmsg(rect_image_buf, "bgr8"))
                    

            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":

    rospy.init_node("upper_arm_hole", anonymous=True)
    
    parser = argparse.ArgumentParser()
    parser.add_argument("module_name", type=str, help="module_name")

    args = parser.parse_args()

    upper_arm_hole = UpperArmHole(args.module_name, "normal")

    upper_arm_hole.run()
    
