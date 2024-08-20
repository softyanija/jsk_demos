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

class CheckLeverCam1():

    def __init__(self):
        self.subs = []
        self.header = None
        self.sub_image = None
        self.sub_hsv_image = None
        self.sub_lever_rect = None
        self.result_image = None
        self.bridge = CvBridge()

        self.width = 320
        self.height = 240
        self.mask_width_half = 8
        self.mask_height_half = 12
        self.mask_delta_x = 0
        self.mask_delta_y = -20
        self.mask_x = 190
        self.mask_y = 150

        self.pub_image = rospy.Publisher('/timer_cam1_rec/lever/around_image', Image, queue_size=1)
        self.pub_mask = rospy.Publisher('/timer_cam1_rec/lever/around_image/debug_image', Image, queue_size=1)
        self.subscribe()


    def subscribe(self):
        sub_socket_rect = message_filters.Subscriber('/timer_cam1_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam1/timer_cam_image/image_rect_color', Image)
        self.subs = [sub_socket_rect, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, sub_socket_rect, image):
        self.sub_socket_rect = socket_rect.rects
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header
    

    def run(self):
        rate = rospy.Rate(2)
        rospy.loginfo("start check_lever_cam2")
    
    if (not rect_msg.rects == []):
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()

                if (not self.sub_socket_rect == []):
                    size_max = 180

                    for i,rect in enumerate(self.sub_socket_rect):
                        size_buf = rect.size.width * rect.size.height
                        if size_buf > size_max and rect.center.x > 0 and rect.center.x < 240 and rect.center.y > 100:
                            size_max = size_buf
                            use_rect = i
                
                        if use_rect is not None:
                            rospy.logino('mask_x is {}, mask_y is {}'.format(rect_msg.rects[use_rect].center.x,rect_msg.rects[use_rect].center.y))
                            mask_x = int(rect_msg.rects[use_rect].center.x)
                            mask_y = int(rect_msg.rects[use_rect].center.y)

                            mask = cv2.rectangle(image_msg_copy, (0,0),(width,height),(0,0,0), -1)
                            mask = cv2.rectangle(image_msg_copy, (mask_x - mask_width_half + mask_delta_x, mask_y - mask_height_half + mask_delta_y), (mask_x + mask_width_half + mask_delta_x, mask_y + mask_height_half + mask_delta_y), (255,255,255), -1)
                            image_msg = cv2.bitwise_and(image_msg, mask)
    
                            #image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")
                            image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")
                            mask = bridge.cv2_to_imgmsg(mask, "bgr8")
    
    pub_image.publish(image_msg)
    pub_mask.publish(mask)






sync = message_filters.ApproximateTimeSynchronizer([sub_img,sub_rect], 10, 0.5)
sync.registerCallback(callback)
rospy.spin()

if __name__ == "__main__":
    rospy.init_node('check_lever_cam1')
    check_lever_cam1 = CheckLeverCam1()
    check_lever_cam1.run()
