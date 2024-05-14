#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import message_filters
import numpy as np
import math
import time
from opencv_apps.msg import Line, LineArrayStamped, RotatedRect, RotatedRectArrayStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

rospy.init_node('indicate_socket_cam1')

def callback(img_msg, rec_msg):

    bridge = CvBridge()
    # memory_under = RotatedRectArrayStamped()
    # memory_under.header = img_msg.header
    
    if (not rec_msg.rects == []):
        size_max = 0.0
        f = 0
        for i,rect in enumerate(rec_msg.rects):
            size_buf = rect.size.width * rect.size.height
            if size_buf > size_max and rect.center.y > 60:
                size_max = size_buf
                use_rect = i
                f = 1

        image_msg = bridge.imgmsg_to_cv2(img_msg, "bgr8")

        if f == 1:
            memory_len = max(rec_msg.rects[use_rect].size.width,rec_msg.rects[use_rect].size.height)
            angle = rec_msg.rects[use_rect].angle
            
            under_x = int(rec_msg.rects[use_rect].center.x - (memory_len / 2)* math.sin(math.radians(angle)))
            under_y = int(rec_msg.rects[use_rect].center.y + (memory_len / 2)* math.cos(math.radians(angle)))
            top_x = int(rec_msg.rects[use_rect].center.x + (memory_len / 2)* math.sin(math.radians(angle)))
            top_y = int(rec_msg.rects[use_rect].center.y - (memory_len / 2)* math.cos(math.radians(angle)))

            lefttop_x = int(rec_msg.rects[use_rect].center.x - rec_msg.rects[use_rect].size.width/2)
            lefttop_y = int(rec_msg.rects[use_rect].center.y - rec_msg.rects[use_rect].size.height/2)
            rightbot_x = int(rec_msg.rects[use_rect].center.x + rec_msg.rects[use_rect].size.width/2)
            rightbot_y = int(rec_msg.rects[use_rect].center.y + rec_msg.rects[use_rect].size.height/2)

            target_x = int(rec_msg.rects[use_rect].center.x)
            target_y = int(rec_msg.rects[use_rect].center.y - rec_msg.rects[use_rect].size.height/2)
            target_top_x = int(rec_msg.rects[use_rect].center.x)
            target_top_y = int(rec_msg.rects[use_rect].center.y - rec_msg.rects[use_rect].size.height*2)
            
            image_msg = cv2.rectangle(image_msg, (lefttop_x, lefttop_y), (rightbot_x, rightbot_y), (0,255,0), 2)
            image_msg = cv2.line(image_msg, (target_top_x,target_top_y),(target_x,target_y),(255,0,0),2)
            image_msg = cv2.circle(image_msg, (target_x, target_y), 3,(0,0,255), 1, 4, 0)
            image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")

        pub_image.publish(image_msg)


pub_image = rospy.Publisher('/timer_cam1_rec/socket/target_point/debug_image', Image, queue_size=1)

sub_img = message_filters.Subscriber('/timer_cam1/timer_cam_image/image_rect_color', Image)
sub_socket = message_filters.Subscriber('/timer_cam1_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)

sync = message_filters.ApproximateTimeSynchronizer([sub_img,sub_socket], 10, 0.5)
sync.registerCallback(callback)
rospy.spin()
