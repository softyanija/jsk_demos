import numpy as np
import rospy
import sys
import tf
import tf2_ros
import time
import math
import cv2
import pdb
from cv_bridge import CvBridge
from numpy import pi
import argparse
import os

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import RotatedRectStamped
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
        self.sub_image = None
        self.gray_img = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.diff_before = None
        self.diff_after = None
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.pub_debug_image_raw = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "debug_image_raw"), Image, queue_size=10)
        self.pub_equ_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "equalizehist_image"), Image, queue_size=10)
        self.pub_threshold_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "threshold_image"), Image, queue_size=10)
        self.pub_ellipse_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "ellipse_image"), Image, queue_size=10)
        self.pub_background_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "background_image"), Image, queue_size=10)
        self.pub_diff_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "diff_image"), Image, queue_size=10)
        self.pub_roi_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "roi_image"), Image, queue_size=10)
        self.pub_masked_image = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "masked_image"), Image, queue_size=10)
        self.pub_upper_arm_hole_result = rospy.Publisher(os.path.join(self.camera, self.recognition_object, "guide_point"), RotatedRectStamped, queue_size=10)

        self.service_name = "/" + camera + "/set_background"
        self.service = rospy.Service(self.service_name, Empty, self.set_backgound_image)

        self.subscribe()


    def subscribe(self):
        sub_image = rospy.Subscriber(self.camera + "/color/image_rect_color", Image, self.cb)

    def set_backgound_image(self, req):
        rospy.loginfo("set background image")
        self.background_image = self.gray_img.copy()
        self.background_is_set = True
        return EmptyResponse()
        
    def cb(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = image.header

    def get_before_image(self):
        while self.diff_before is None:
            if (self.sub_image is not None):
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
        
                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                self.diff_before = equ_img_clip

                
    def get_after_image(self):
        while self.diff_after is None:
            if (self.sub_image is not None):
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
        
                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                self.diff_after = equ_img_clip

                
    def set_part_roi(self):
        diff = cv2.absdiff(self.diff_before, self.diff_after)
        cv2.imwrite("/home/amabe/rosbag/diff.png", diff)
        ret, diff = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)
        cv2.imwrite("/home/amabe/rosbag/diff_th.png", diff)

        
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

            self.capture_hole = True
            if((self.sub_image is not None) and (self.capture_hole == True)):
                #pdb.set_trace()
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
                self.gray_img = gray_img.copy()

                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                ret, threshold_image = cv2.threshold(equ_img_clip, 120, 255, cv2.THRESH_BINARY)

                self.pub_debug_image_raw.publish(self.bridge.cv2_to_imgmsg(self.sub_image, "bgr8"))
                if self.debug_mode == "debug":
                    self.pub_equ_image.publish(self.bridge.cv2_to_imgmsg(equ_img_clip))
                    self.pub_threshold_image.publish(self.bridge.cv2_to_imgmsg(threshold_image))

                if self.background_is_set:
                    diff = cv2.absdiff(self.background_image, gray_img)
                    _, diff_thresholded = cv2.threshold(diff, 70, 255, cv2.THRESH_BINARY)
                    diff_thresholded = cv2.medianBlur(diff_thresholded, 11)
                    diff_contours, hierarchy = cv2.findContours(diff_thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    if self.debug_mode == "debug":
                        self.pub_diff_image.publish(self.bridge.cv2_to_imgmsg(diff))
                        
                    roi_image = cv2.cvtColor(diff_thresholded.copy(), cv2.COLOR_GRAY2BGR)
                    
                    size_buff = 0
                    index_buff = None
                    for i, cnt in enumerate(diff_contours):
                        x, y, width, height = cv2.boundingRect(cnt)
                        if (size_buff < width * height) and (y < self.background_image.shape[1]//2):
                            size_buff = width * height
                            index_buff = i

                    if index_buff is not None:
                        x, y, width, height = cv2.boundingRect(diff_contours[index_buff])
                        cv2.rectangle(roi_image, (x, y), (x + width, y + height), color=(0, 255, 0), thickness=4)


                        mask_roi = np.zeros_like(diff_thresholded)
                        mask_roi = cv2.rectangle(mask_roi, (x, y), (x + width, y + height), 255, cv2.FILLED)
                        mask = cv2.bitwise_and(diff_thresholded, mask_roi)
                        masked_image = cv2.bitwise_and(threshold_image, mask_roi)
                        if self.debug_mode == "debug":
                            self.pub_roi_image.publish(self.bridge.cv2_to_imgmsg(roi_image, "bgr8"))
                            self.pub_masked_image.publish(self.bridge.cv2_to_imgmsg(masked_image))
                    
                        ellipse_contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                        ellipse_drawed_image = self.sub_image.copy()
                        ellipse_list = []
                        x_range_min = 10
                        x_range_max = 250
                        cv2.rectangle(ellipse_drawed_image, (x + x_range_min, y + height//10), (x + x_range_max, y + height*9//10), (255, 0, 0), 4, cv2.LINE_AA)
                        ellipse_max_index = None
                        ellipse_max_size = 0
                        for i, cnt in enumerate(ellipse_contours):
                            if len(cnt) >= 5: 
                                ellipse = cv2.fitEllipse(cnt)
                                if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                                    ellipse_list.append(ellipse)
                                    #nan is not eliminated
                                    cx = int(ellipse[0][0])
                                    cy = int(ellipse[0][1])
                                    w = int(ellipse[1][0])
                                    h = int(ellipse[1][1])
                                    if (h * w > 300) and (h * w < 4000) and (cx > x + x_range_min) and (cx < x + x_range_max) and (cy > y + height//10) and (cy < y + height*9//10):
                                        if h * w > ellipse_max_size:
                                            ellipse_max_size = h * w
                                            ellipse_max_index = i

                        if ellipse_max_index is not None:
                            ellipse = cv2.fitEllipse(ellipse_contours[ellipse_max_index])

                            ellipse_msg = RotatedRectStamped()
                            ellipse_msg.header.stamp = rospy.Time.now()
                            ellipse_msg.rect.x = ellipse[0][0]
                            ellipse_msg.rect.y = ellipse[0][1]
                            
                            ellipse_msg.rect.width = ellipse[1][0]
                            ellipse_msg.rect.height = ellipse[1][1]
                            ellipse_msg.rect.angle = ellipse[2]

                            self.pub_upper_arm_hole_result.publish(ellipse_msg)

                            ellipse_drawed_image  = cv2.ellipse(ellipse_drawed_image, ellipse, (0, 255, 0), 3)

                        self.pub_ellipse_image.publish(self.bridge.cv2_to_imgmsg(ellipse_drawed_image, "bgr8"))

                    
                    if self.background_image is not None:
                        self.pub_background_image.publish(self.bridge.cv2_to_imgmsg(self.background_image))
                
            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("module_name", type=str, help="module_name")
    parser.add_argument("--debug", type=str, help="flag to launch in debug mode")
    
    rospy.init_node("upper_arm_hole", anonymous=True)

    args = parser.parse_args()

    if args.debug == "True" or args.debug == "true":
        print("launch in debug mode")
        upper_arm_hole = UpperArmHole(args.module_name, "debug")
    else:
        print("launch in normal mode")
        upper_arm_hole = UpperArmHole(args.module_name, "normal")

    upper_arm_hole.run()
    
