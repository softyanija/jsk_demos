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

rospy.init_node('indicate_under')

def callback(img_msg, rec_msg):
    
    bridge = CvBridge()
    memory_under = PoseArray()
    memory_under.header = img_msg.header
    
    if (not rec_msg.rects == []):
        size_max = 0.0
        f = 0
        for i,rect in enumerate(rec_msg.rects):
            size_buf = rect.size.width * rect.size.height
            if size_buf > size_max and rect.center.y > 40:
                size_max = size_buf
                use_rect = i
                f = 1

        under = Pose()
        image_msg = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        if f == 1:
            memory_len = max(rec_msg.rects[use_rect].size.width,rec_msg.rects[use_rect].size.height)
            #memory_len = rec_msg.rects[use_rect].size.width
            # under_x = int(rec_msg.rects[use_rect].center.x)
            # under_y = int(rec_msg.rects[use_rect].center.y + memory_len/2)
            
            if (abs(rec_msg.rects[use_rect].angle) < (rec_msg.rects[use_rect].angle + 90)):
                angle = rec_msg.rects[use_rect].angle
            else:
                angle = (rec_msg.rects[use_rect].angle + 90)

            print(angle)
            
            under_x = int(rec_msg.rects[use_rect].center.x + (memory_len / 2)* math.sin(angle))
            under_y = int(rec_msg.rects[use_rect].center.y + (memory_len / 2)* math.cos(angle))
            top_x = int(rec_msg.rects[use_rect].center.x - (memory_len / 2)* math.sin(angle))
            top_y = int(rec_msg.rects[use_rect].center.y - (memory_len / 2)* math.cos(angle))
            image_msg = cv2.circle(image_msg, (under_x,under_y), 3,(0,0,255),1,4,0)
            #image_msg = cv2.circle(image_msg, (top_x,top_y), 5,(255,0,0),2,4,0)
            #image_msg = cv2.line(image_msg, (under_x,under_y),(top_x,top_y),(0,255,0),1)
            
        
    image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")

    pub_under.publish(memory_under)
    pub_image.publish(image_msg)


pub_under = rospy.Publisher('/timer_cam1_rec/memory/memory_under', PoseArray, queue_size=1)
pub_image = rospy.Publisher('/timer_cam1_rec/memory/memory_under/debug_image', Image, queue_size=1)

sub_img = message_filters.Subscriber('/timer_cam1/timer_cam_image/image_rect_color', Image)
sub_rec = message_filters.Subscriber('/timer_cam1_rec/memory/general_contours/rectangles', RotatedRectArrayStamped)

sync = message_filters.ApproximateTimeSynchronizer([sub_img,sub_rec], 10, 0.5)
sync.registerCallback(callback)
rospy.spin()
