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
        self.sub_hsv_image = None
        self.background_image = None
        self.gray_img = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False

        self.parts_detect_rect = ((300, 50), (650, 300))

        self.pub_debug_depth = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "debug_depth"), Image, queue_size=10)
        self.pub_backround_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "background_image"), Image, queue_size=10)
        self.pub_diff_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "diff_image"), Image, queue_size=10)
        self.pub_diff_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "diff_image"), Image, queue_size=10)
        self.pub_masked_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "masked_image"), Image, queue_size=10)
        self.pub_rect_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "rect_image"), Image, queue_size=10)
        self.pub_upper_arm_hole_result = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "guide_point"), RotatedRectStamped, queue_size=10)
        self.pub_upper_arm_hole_result_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "guide_point", "image"), Image, queue_size=10)

        self.service_name = "/" + camera + "/set_background"
        self.service = rospy.Service(self.service_name, Empty, self.set_backgound_image)

        self.multi_subscribe()
        self.rects_subscribe()
        self.morphology_subscribe()
        self.hsv_subscribe()


    def multi_subscribe(self):
        sub_color_image = message_filters.Subscriber(self.camera + "/color/image_rect_color/republish", Image)
        sub_depth_image = message_filters.Subscriber(self.camera + "/aligned_depth_to_color/image_raw/republish", Image)
        sub_mask_image = message_filters.Subscriber(self.camera + "/" + self.recognition_object + "_pre" + "/multiply_mask_image/output", Image)

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


    def hsv_subscribe(self):
        rospy.Subscriber(self.camera + "/" + self.recognition_object +  "/hsv_color_filter/image", Image, self.hsv_callback)


    def hsv_callback(self, image):
        self.sub_hsv_image = self.bridge.imgmsg_to_cv2(image, "8UC1")


    def set_backgound_image(self, req):
        rospy.loginfo("set background image")
        self.background_image = self.sub_mask_image.copy()
        self.background_is_set = True
        return EmptyResponse()

    def sort_box(self, box):
        min_index = np.argmin((np.sum(box, axis=1)))
        sorted_box = []
        for i in range(box.shape[0]):
            if i == 0:
                sorted_box = box[(min_index + i) % box.shape[0]].reshape(1,2)
            else:
                sorted_box = np.append(sorted_box, box[(min_index + i) % box.shape[0]].reshape(1,2), axis=0)
                
        return sorted_box


    def return_max_ellipe(self, reference, box):
        guide_ellipse = None
        drawed_image  = self.sub_color_image.copy()
        ellipse_contours, _ = cv2.findContours(reference, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ellipse_list = []
        ellipse_max_index = None
        ellipse_max_size = 0
        ellipse_limit_max = 800
        ellipse_limit_min = 100
        box = self.sort_box(box)
        box_vec_x = box[1] - box[0]
        box_vec_y = box[3] - box[0]
        # limit = ((0, 0.5), (0.4, 1))
        limit = ((0, 0.5), (0, 1))
        limit_box = np.array((box[0] + box_vec_x * limit[0][0] + box_vec_y * limit[1][0],
                              box[0] + box_vec_x * limit[0][1] + box_vec_y * limit[1][0],
                              box[0] + box_vec_x * limit[0][1] + box_vec_y * limit[1][1],
                              box[0] + box_vec_x * limit[0][0] + box_vec_y * limit[1][1])).astype(int)

        drawed_image = cv2.drawContours(drawed_image, [limit_box], 0, (255, 0 ,0) , 2)

        for i, cnt in enumerate(ellipse_contours):
            if len(cnt) >= 5:
                ellipse = cv2.fitEllipse(cnt)
                if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0]))and (not math.isnan(ellipse[1][1])):
                    cx = int(ellipse[0][0])
                    cy = int(ellipse[0][1])
                    w = int(ellipse[1][0])
                    h = int(ellipse[1][1])
                    drawed_image = cv2.ellipse(drawed_image, ellipse, (0, 255, 0), 1)
                    if (h * w > ellipse_max_size) and (h * w < ellipse_limit_max) and (cx > limit_box[0][0]) and (cx < limit_box[2][0]) and (cy > limit_box[0][1]) and (cy < limit_box[2][1]):
                        ellipse_max_size = h * w
                        ellipse_max_index = i

        if ellipse_max_size > ellipse_limit_min:
            guide_ellipse = cv2.fitEllipse(ellipse_contours[ellipse_max_index])
            drawed_image = cv2.ellipse(drawed_image, guide_ellipse, (0, 0, 255), 3)

        return guide_ellipse, drawed_image

        
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
                self.pub_debug_depth.publish(self.bridge.cv2_to_imgmsg(self.sub_depth_image))
                
                if self.background_is_set:

                    diff = cv2.absdiff(self.background_image, self.sub_mask_image)
                    diff_msg = self.bridge.cv2_to_imgmsg(diff, "mono8")
                    diff_msg.header = self.header
                    self.pub_diff_image.publish(diff_msg)
                    self.pub_backround_image.publish(self.bridge.cv2_to_imgmsg(self.background_image))
                    ellipse = None

                    mask_roi = np.zeros_like(self.sub_morphology_image)
                    masked_image = np.zeros_like(self.sub_morphology_image)
                    upper_arm_hole_result_image  = self.sub_color_image.copy()

                    if self.sub_parts_rect is not None:

                        rect_image_buf = self.sub_color_image.copy()
                        contours, _ = cv2.findContours(self.sub_morphology_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        rect_image_buf = cv2.rectangle(rect_image_buf, self.parts_detect_rect[0],self.parts_detect_rect[1], (0, 255, 0), thickness=2)
                        max_size = 0
                        max_index = None
                        max_box = None
                        
                        for i, cnt in enumerate(contours):
                            rect = cv2.minAreaRect(cnt)
                            box = cv2.boxPoints(rect)
                            box = np.intp(box)
                            
                            if (rect[0][0] > self.parts_detect_rect[0][0]) and (rect[0][0] < self.parts_detect_rect[1][0]) and (rect[0][1] > self.parts_detect_rect[0][1]) and (rect[0][1] < self.parts_detect_rect[1][1]) and (rect[1][0] * rect[1][1] > 300):
                                if rect[1][0] * rect[1][1] > max_size:
                                    max_index = i
                                    max_size = rect[1][0] * rect[1][1]
                                    max_box = box
                                    
                                rect_image_buf = cv2.drawContours(rect_image_buf,[box],0,(0,0,255),1)

                            else:
                                rect_image_buf = cv2.drawContours(rect_image_buf,[box],0,(255,0,0),1)

                        morphology_image_buf = self.sub_morphology_image
                        
                        if max_box is not None:
                            rect_image_buf = cv2.drawContours(rect_image_buf,[max_box],0,(0,0,255),3)
                            mask_roi = cv2.drawContours(mask_roi, [max_box], 0,255, cv2.FILLED)

                            masked_image =cv2.bitwise_and(self.sub_hsv_image, mask_roi)
                            ellipse, upper_arm_hole_result_image = self.return_max_ellipe(masked_image, max_box)

                        if ellipse is not None:
                            guide_point_msg = RotatedRectStamped()
                            guide_point_msg.header.stamp = rospy.Time.now()
                            guide_point_msg.rect.x = ellipse[0][0]
                            guide_point_msg.rect.y = ellipse[0][1]
                            
                            guide_point_msg.rect.width = ellipse[1][0]
                            guide_point_msg.rect.height = ellipse[1][1]
                            guide_point_msg.rect.angle = ellipse[2]                            
                            self.pub_upper_arm_hole_result.publish(guide_point_msg)
                        self.pub_masked_image.publish(self.bridge.cv2_to_imgmsg(masked_image, "mono8"))
                        self.pub_upper_arm_hole_result_image.publish(self.bridge.cv2_to_imgmsg(upper_arm_hole_result_image, "bgr8"))
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
    
