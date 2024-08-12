#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import message_filters
import numpy as np
import math
import time
from opencv_apps.msg import Line,LineArrayStamped,RotatedRectArrayStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class CheckLeverCam2():

    def __init__(self):
        self.subs = []
        self.header = None
        self.lines_msg = None
        self.sub_image = None
        self.sub_lever_rect = None
        self.memory_under = None
        self.result_image = None
        self.socket_angle = None
        
        self.width = 640
        self.height = 480
        self.mask_width_half = 16
        self.mask_height_half = 16
        self.mask_delta_x = -6
        self.mask_delta_y = -40
        self.mask_x = 380
        self.mask_y = 380
        
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/timer_cam2_rec/lever/around_image/debug_image', Image, queue_size=1)
        self.pub_mask = rospy.Publisher('/timer_cam2_rec/lever/around_image/mask', Image, queue_size=1)
        self.subscribe()


    def subscribe(self):
        rospy.loginfo("hoge")
        sub_socket_rect = message_filters.Subscriber('/timer_cam2_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)
        self.subs = [sub_socket_rect, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, socket_rect, image):
        self.sub_socket_rect = socket_rect.rects
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header


    def run(self):
        rate = rospy.Rate(2)
        rospy.loginfo("start check_lever_cam2")
        # global mask_x
        # global mask_y
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                self.sub_image_copy = self.sub_image.copy()
                self.memory_angle = None
                self.socket_angle = None

                if (not self.sub_socket_rect == []):
                    size_max = 80
                    use_rect = None
                    for i,rect in enumerate(self.sub_socket_rect):
                        size_buf = rect.size.width * rect.size.height
                        if size_buf > size_max and rect.center.x > 320 and rect.center.x < 480 and rect.center.y > 300:
                            size_max = size_buf
                            use_rect = i
                
                    if use_rect is not None:
                        rospy.loginfo("mask_x is {}, mask_y is {}".format(self.sub_socket_rect[use_rect].center.x, self.sub_socket_rect[use_rect].center.y))
                        
                        mask_x = int(self.sub_socket_rect[use_rect].center.x)
                        mask_y = int(self.sub_socket_rect[use_rect].center.y)

                        self.mask = cv2.rectangle(self.sub_image_copy, (0,0),(self.width,self.height),(0,0,0), -1)
                        self.mask = cv2.rectangle(self.sub_image_copy, (self.mask_x - self.mask_width_half + self.mask_delta_x, self.mask_y - self.mask_height_half + self.mask_delta_y), (self.mask_x + self.mask_width_half + self.mask_delta_x, self.mask_y + self.mask_height_half + self.mask_delta_y), (255,255,255), -1)
                        self.result_image = cv2.bitwise_and(self.result_image, self.mask)
    
                #image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")
                self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
                self.mask = self.bridge.cv2_to_imgmsg(self.mask, "bgr8")
    
                self.pub_image.publish(self.result_image)
                self.pub_mask.publish(self.mask)

            else:
                rospy.loginfo("didn't recieve image")


if __name__ == "__main__":
    rospy.init_node('check_lever_cam2')
    check_lever_cam2 = CheckLeverCam2()
    check_lever_cam2.run()
