#!/usr/bin/env python3

import numpy as np
import rospy
import time
import math
import cv2
import pdb
import message_filters
from cv_bridge import CvBridge
from numpy import pi
import argparse
import rospkg

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import RotatedRectStamped
from jsk_recognition_msgs.msg import RectArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from multi_device_view.msg import Ellipse

rospack = rospkg.RosPack()

class ServoGear():

    def __init__(self,
                 camera,
                 debug_mode,
                 **kwargs):
        
        self.camera = camera
        self.recognition_object = "servo_gear"
        self.debug_mode = False
        self.sub_color_image = None
        self.sub_depth_image = None
        self.sub_mask_image = None
        self.roi = None
        self.roi_top_offset = 10
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.diff_before = None
        self.diff_after = None
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.reference_image_path = rospack.get_path("multi_device_view") + "/scripts/template_image/servo_gear_template.png"
        self.pub_debug_image_raw = rospy.Publisher(self.camera + "/" + self.recognition_object + "/debug_image_raw", Image, queue_size=10)
        self.pub_debug_image_matching = rospy.Publisher(self.camera + "/" + self.recognition_object + "/debug_image_matching", Image, queue_size=10)
        self.pub_servo_gear_result = rospy.Publisher(self.camera + "/" + self.recognition_object + "/target_point", RotatedRectStamped, queue_size=10)
        
        self.subscribe()


    def subscribe(self):
        sub_color_image = message_filters.Subscriber(self.camera + "/color/image_rect_color", Image)
        sub_depth_image = message_filters.Subscriber(self.camera + "/depth/image_rect_raw", Image)
        sub_mask_image = message_filters.Subscriber(self.camera + "/" + self.recognition_object + "/multiply_mask_image/output", Image)
        sub_roi = message_filters.Subscriber(self.camera + "/" + self.recognition_object + "/mask_image_to_rect/output", RectArray)
        self.subs = [sub_color_image, sub_depth_image, sub_mask_image, sub_roi]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, color, depth, mask, roi):
        self.sub_color_image = self.bridge.imgmsg_to_cv2(color, "bgr8")
        self.sub_depth_image = self.bridge.imgmsg_to_cv2(depth, "16UC1")
        self.sub_mask_image = self.bridge.imgmsg_to_cv2(mask, "16UC1")
        self.roi = roi.rects
        self.header = color.header


    def run(self):
        rate = rospy.Rate(5)

        rospy.loginfo("start to recognize servo gear")

        reference_image = cv2.imread(self.reference_image_path)
        rospy.loginfo("reading template from {}".format(self.reference_image_path))
        reference_h, reference_w, _ = reference_image.shape
        threshold = 0.93
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
            
            if (self.sub_color_image is not None) and (self.sub_depth_image is not None) and (self.sub_mask_image is not None):

                #roi_x, roi_y, roi_w, roi_h = 83, 79, 750, 284
                roi = self.roi[0]
                roi_x, roi_y, roi_w, roi_h = roi.x, roi.y - self.roi_top_offset, roi.width, roi.height 
                cliped_image = self.sub_color_image.copy()[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

                result = None
                result_image = self.sub_color_image.copy()
                result = cv2.matchTemplate(cliped_image, reference_image, cv2.TM_CCOEFF_NORMED)
                #result = cv2.matchTemplate(self.ub_color_image, reference_image, cv2.TM_CCORR_NORMED)

                if result is not None:
                    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
                    top_left = (max_loc[0] + roi_x, max_loc[1] + roi_y)
                    bottom_right = (top_left[0] + reference_w, top_left[1] + reference_h)
                    target_point = (top_left[0] + reference_w // 2, top_left[1] )
                
                    cv2.rectangle(result_image, top_left, bottom_right, (255, 255, 0), 1)
                    cv2.rectangle(result_image, (roi_x, roi_y), (roi_x + roi_w, roi_y + roi_h), (0, 0, 255), 2)
                    cv2.circle(result_image, target_point, 2, (0, 0, 255), -1)

                    target_point = RotatedRectStamped()
                    target_point.header.stamp = rospy.Time.now()

                    target_point.rect.x = top_left[0]
                    target_point.rect.y = top_left[1]

                    target_point.rect.width = reference_w
                    target_point.rect.height = reference_h
                    target_point.rect.angle = 0

                    self.pub_servo_gear_result.publish(target_point)

                self.pub_debug_image_raw.publish(self.bridge.cv2_to_imgmsg(self.sub_color_image, "bgr8"))
                self.pub_debug_image_matching.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
                
            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("module_name", type=str, help="module_name")  
    rospy.init_node("servo_gear", anonymous=True)

    args = parser.parse_args()
    
    servo_gear = ServoGear(args.module_name, "normal")

    servo_gear.run()
