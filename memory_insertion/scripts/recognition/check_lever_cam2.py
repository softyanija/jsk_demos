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
        self.sub_image = None
        self.sub_hsv_image = None
        self.sub_lever_rect = None
        self.result_image = None
        
        self.width = 320
        self.height = 240
        self.mask_width_half = 12
        self.mask_height_half = 14
        self.mask_delta_x = -6
        self.mask_delta_y = -23
        self.mask_x = 190
        self.mask_y = 190
        
        self.bridge = CvBridge()
        self.pub_image = rospy.Publisher('/timer_cam2_rec/lever/around_image/debug_image', Image, queue_size=1)
        self.pub_mask = rospy.Publisher('/timer_cam2_rec/lever/around_image/mask', Image, queue_size=1)
        self.subscribe()


    def subscribe(self):
        sub_socket_rect = message_filters.Subscriber('/timer_cam2_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_hsv = message_filters.Subscriber('/timer_cam2_rec/lever/hsv_color_filter/image', Image)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)
        self.subs = [sub_socket_rect, sub_hsv, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, socket_rect, hsv, image):
        self.sub_socket_rect = socket_rect.rects
        self.sub_hsv_image = self.bridge.imgmsg_to_cv2(hsv, "rgb8")
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

                if (not self.sub_socket_rect == []):
                    size_max = 20
                    use_rect = None
                    for i,rect in enumerate(self.sub_socket_rect):
                        size_buf = rect.size.width * rect.size.height
                        if size_buf > size_max and rect.center.x > 160 and rect.center.x < 240 and rect.center.y > 150:
                            size_max = size_buf
                            use_rect = i
                
                    if use_rect is not None:
                        rospy.loginfo("mask_x is {}, mask_y is {}".format(self.sub_socket_rect[use_rect].center.x, self.sub_socket_rect[use_rect].center.y))
                        
                        mask_x = int(self.sub_socket_rect[use_rect].center.x)
                        mask_y = int(self.sub_socket_rect[use_rect].center.y)

                        self.mask = np.zeros_like(self.sub_image)
                        self.mask = cv2.rectangle(self.mask, (self.mask_x - self.mask_width_half + self.mask_delta_x, self.mask_y - self.mask_height_half + self.mask_delta_y), (self.mask_x + self.mask_width_half + self.mask_delta_x, self.mask_y + self.mask_height_half + self.mask_delta_y), (255,255,255), -1)
                        self.result_image = cv2.bitwise_and(self.result_image, self.mask)
                        self.sub_hsv_image = cv2.bitwise_and(self.sub_hsv_image, self.mask)
                        image_gray = cv2.cvtColor(self.sub_hsv_image, cv2.COLOR_BGR2GRAY)

                        ret,thresh = cv2.threshold(image_gray,127,255,0)
                        contours,hierarchy = cv2.findContours(thresh, 1, 2)
                      
                        if (not contours == ()):
                            cnt = max(contours, key=lambda x: cv2.contourArea(x))
                            rect = cv2.minAreaRect(cnt)
                            box = cv2.boxPoints(rect)
                            box = np.int0(box)

                            self.result_image = cv2.rectangle(self.result_image, (box[0][0], box[0][1]), (box[2][0], box[2][1]), (0,0,255), 2, )


                self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
                self.mask = self.bridge.cv2_to_imgmsg(self.mask, "rgb8")
    
                # self.pub_image.publish(self.result_image)
                self.pub_image.publish(self.result_image)
                self.pub_mask.publish(self.mask)

            else:
                rospy.loginfo("didn't recieve image")


if __name__ == "__main__":
    rospy.init_node('check_lever_cam2')
    check_lever_cam2 = CheckLeverCam2()
    check_lever_cam2.run()
