#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import message_filters
import numpy as np
import math
import time
import pdb
from opencv_apps.msg import Line,LineArray,LineArrayStamped,RotatedRect,RotatedRectArrayStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class LeverCheck():

    def __init__(self):
        self.sub_rects = None
        self.sub_image = None
        self.debug_image = None
        self.header = None
        self.target_point = None
        self.target_point_buf = None
        self.lever_range_lefttop = (175, 150)
        self.lever_range_rightbot = (200, 180)
        self.bridge = CvBridge()
        # self.pub_target_point = rospy.Publisher('/timer_cam2_rec/socket/target_point', RotatedRectArrayStamped, queue_size=1)
        self.pub_debug_image = rospy.Publisher('/timer_cam2_rec/lever/lever_check/debug_image', Image, queue_size=1)

        self.multi_subscribe()


    def multi_subscribe(self):
        sub_rects = message_filters.Subscriber('/timer_cam2_rec/lever/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam2_rec/lever/draw_rects/output', Image)
        self.subs = [sub_rects, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.multi_callback)


    def multi_callback(self, rects, image):
        self.sub_rects = rects  
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = image.header

    def run(self):
        rate = rospy.Rate(5)
        self.lever_check = RotatedRectArrayStamped()

        rospy.loginfo("start to recognition socket position")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
                
            if self.sub_image is not None:
                self.debug_image = self.sub_image.copy()
                # self.debug_image = cv2.rectangle(self.debug_image, self.lever_range_lefttop, self.lever_range_rightbot, (0,0,255), 1)

                if (not self.sub_rects.rects == []):
                    self.lever_check_buf = RotatedRect()
                    size_max = 20
                    f = 0
                    for i,rect in enumerate(self.sub_rects.rects):
                        size_buf = rect.size.width * rect.size.height

                        if size_buf > size_max:
                            size_max = size_buf
                            use_rect = i
                            f = 1

                    if f == 1:
                        socket_len = max(self.sub_rects.rects[use_rect].size.width, self.sub_rects.rects[use_rect].size.height)
                        angle = self.sub_rects.rects[use_rect].angle
                        lefttop_x = int(self.sub_rects.rects[use_rect].center.x - self.sub_rects.rects[use_rect].size.width/2)
                        lefttop_y = int(self.sub_rects.rects[use_rect].center.y - self.sub_rects.rects[use_rect].size.height/2)
                        rightbot_x = int(self.sub_rects.rects[use_rect].center.x + self.sub_rects.rects[use_rect].size.width/2)
                        rightbot_y = int(self.sub_rects.rects[use_rect].center.y + self.sub_rects.rects[use_rect].size.height/2)
            
                        self.lever_check_buf.angle = self.sub_rects.rects[use_rect].angle
                        self.lever_check_buf.center.x = self.sub_rects.rects[use_rect].center.x + (socket_len / 2)* math.sin(math.radians(angle))
                        self.lever_check_buf.center.y = self.sub_rects.rects[use_rect].center.y + (socket_len / 2)* math.cos(math.radians(angle))
                        self.lever_check_buf.size.width = self.sub_rects.rects[use_rect].size.width
                        self.lever_check_buf.size.height = self.sub_rects.rects[use_rect].size.height

                        self.lever_check.rects.append(self.lever_check_buf)

                        self.debug_image = cv2.rectangle(self.debug_image, (lefttop_x, lefttop_y), (rightbot_x, rightbot_y), (255,0,0), 2)
                        self.lever_check.header = self.header
                        # self.pub_target_point.publish(self.target_point)

                self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(self.debug_image, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("lever_check_cam2")
    lever_check = LeverCheck()
    lever_check.run()
    
