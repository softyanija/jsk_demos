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


class SocketPosition():

    def __init__(self):
        self.sub_rects = None
        self.sub_image = None
        self.debug_image = None
        self.header = None
        self.target_point = None
        self.target_point_buf = None
        self.offset_x = 3
        self.offset_y = 10
        self.bridge = CvBridge()
        self.pub_target_point = rospy.Publisher('/timer_cam2_rec/socket/target_point', RotatedRectArrayStamped, queue_size=1)
        self.pub_debug_image = rospy.Publisher('/timer_cam2_rec/socket/target_point/debug_image', Image, queue_size=1)

        self.multi_subscribe()


    def multi_subscribe(self):
        sub_rects = message_filters.Subscriber('/timer_cam2_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)
        self.subs = [sub_rects, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.multi_callback)


    def multi_callback(self, rects, image):
        self.sub_rects = rects  
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = image.header


    def run(self):
        rate = rospy.Rate(5)
        self.target_point = RotatedRectArrayStamped()

        rospy.loginfo("start to recognition socket position")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
                
            if self.sub_image is not None:
                self.debug_image = self.sub_image.copy()

                if (not self.sub_rects.rects == []):
                    self.target_point_buf = RotatedRect()
                    size_max = 0.0
                    f = 0
                    for i,rect in enumerate(self.sub_rects.rects):
                        size_buf = rect.size.width * rect.size.height

                        if size_buf > size_max and rect.center.x > 130 and rect.center.y > 120:
                            size_max = size_buf
                            use_rect = i
                            f = 1

                    if f == 1:
                        socket_len = max(self.sub_rects.rects[use_rect].size.width, self.sub_rects.rects[use_rect].size.height)
                        angle = self.sub_rects.rects[use_rect].angle

                        under_x = int(self.sub_rects.rects[use_rect].center.x - self.sub_rects.rects[use_rect].size.width/2 - (socket_len / 2)* math.sin(math.radians(angle)) - self.offset_x)
                        under_y = int(self.sub_rects.rects[use_rect].center.y - self.sub_rects.rects[use_rect].size.height + (socket_len / 2)* math.cos(math.radians(angle)) - self.offset_y)
                        top_x = int(self.sub_rects.rects[use_rect].center.x - self.sub_rects.rects[use_rect].size.width/2 + (socket_len / 2)* math.sin(math.radians(angle)) - self.offset_x)
                        top_y = int(self.sub_rects.rects[use_rect].center.y - self.sub_rects.rects[use_rect].size.height - (socket_len / 2)* math.cos(math.radians(angle)) - self.offset_y)

                        lefttop_x = int(self.sub_rects.rects[use_rect].center.x - self.sub_rects.rects[use_rect].size.width/2)
                        lefttop_y = int(self.sub_rects.rects[use_rect].center.y - self.sub_rects.rects[use_rect].size.height/2)
                        rightbot_x = int(self.sub_rects.rects[use_rect].center.x + self.sub_rects.rects[use_rect].size.width/2)
                        rightbot_y = int(self.sub_rects.rects[use_rect].center.y + self.sub_rects.rects[use_rect].size.height/2)
            
                        self.target_point_buf.angle = self.sub_rects.rects[use_rect].angle
                        self.target_point_buf.center.x = self.sub_rects.rects[use_rect].center.x + (socket_len / 2)* math.sin(math.radians(angle))
                        self.target_point_buf.center.y = self.sub_rects.rects[use_rect].center.y + (socket_len / 2)* math.cos(math.radians(angle))
                        self.target_point_buf.size.width = self.sub_rects.rects[use_rect].size.width
                        self.target_point_buf.size.height = self.sub_rects.rects[use_rect].size.height

                        self.target_point.rects.append(self.target_point_buf)

                        self.debug_image = cv2.rectangle(self.debug_image, (lefttop_x, lefttop_y), (rightbot_x, rightbot_y), (0,255,0), 2)
                        self.debug_image = cv2.circle(self.debug_image, (under_x,under_y), 3,(255,0,0),2,3,0)
                        self.debug_image = cv2.line(self.debug_image, (under_x,under_y), (top_x,top_y),(0,0,255),2)
                        self.target_point.header = self.header
                        self.pub_target_point.publish(self.target_point)

                self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(self.debug_image, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("socket_position_cam2")
    socket_position = SocketPosition()
    socket_position.run()
    
