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

rospy.init_node('check_lever_cam2')

width = 320
height = 240
mask_width_half = 8
mask_height_half = 8
mask_delta_x = -3
mask_delta_y = -20
mask_x = 190
mask_y = 190

def callback(img_msg, rect_msg):

    bridge = CvBridge()

    image_msg = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    image_msg_copy = image_msg.copy()

    if (not rect_msg.rects == []):
        size_max = 20
        f = 0
        for i,rect in enumerate(rect_msg.rects):
            size_buf = rect.size.width * rect.size.height
            if size_buf > size_max and rect.center.x > 160 and rect.center.x < 240 and rect.center.y > 150:
                size_max = size_buf
                use_rect = i
                f = 1
                
        if f == 1:
            print('mask_x is {}, mask_y is {}'.format(rect_msg.rects[use_rect].center.x,rect_msg.rects[use_rect].center.y))
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

pub_image = rospy.Publisher('/timer_cam2_rec/lever/around_image', Image, queue_size=1)
pub_mask = rospy.Publisher('/timer_cam2_rec/lever/around_image/debug_image', Image, queue_size=1)

sub_img = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)
sub_rect = message_filters.Subscriber('/timer_cam2_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)

sync = message_filters.ApproximateTimeSynchronizer([sub_img,sub_rect], 10, 0.5)
sync.registerCallback(callback)
rospy.spin()
