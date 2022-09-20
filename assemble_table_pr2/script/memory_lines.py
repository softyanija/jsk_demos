#!/usr/bin/env python

import cv2
import cv_bridge
import message_filters
import numpy as np
import rospy
import math
import time
from opencv_apps.msg import Line,LineArrayStamped
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('detect_lines')

def callback(lines_msg, area_msg, image_msg):

    bridge = CvBridge()
    memory_line = LineArrayStamped()
    memory_line.header = lines_msg.header

    if ((not lines_msg.lines == []) and (not area_msg.poses == [])):
        
        l1 = (area_msg.poses[1].position.x - area_msg.poses[0].position.x)**2 + (area_msg.poses[1].position.y - area_msg.poses[0].position.y)**2
        l2 = (area_msg.poses[1].position.x - area_msg.poses[2].position.x)**2 + (area_msg.poses[1].position.y - area_msg.poses[2].position.y)**2

        if l1 > l2:
            t = (area_msg.poses[1].position.y - area_msg.poses[0].position.y) / (area_msg.poses[1].position.x - area_msg.poses[0].position.x)
        else:
            t = (area_msg.poses[1].position.y - area_msg.poses[2].position.y) / (area_msg.poses[1].position.x - area_msg.poses[2].position.x)

        ref_deg = math.atan(t)
        use_line = 0
        
        for i,line in enumerate(lines_msg.lines):
            buf = (line.pt1.y - line.pt2.y) / (line.pt1.x - line.pt2.x)
            buf_deg = math.atan(buf)
            diff_deg = buf_deg - ref_deg

            if i == 0:
                diff_deg_min = diff_deg
                line_start = (int(line.pt1.x), int(line.pt1.y))
                line_end = (int(line.pt2.x), int(line.pt2.y))

            if diff_deg_min > diff_deg:
                diff_deg_min = diff_deg
                use_line = i
                line_start = (int(line.pt1.x), int(line.pt1.y))
                line_end = (int(line.pt2.x), int(line.pt2.y))

        line = Line()
        image_msg = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        if (abs(diff_deg_min) < 0.2):
            line = lines_msg.lines[use_line]
            memory_line.lines.append(line)
            image_msg = cv2.line(image_msg, line_start, line_end, (0,255,0), 3)
        image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")
    
    pub_line.publish(memory_line)
    pub_image.publish(image_msg)


pub_line = rospy.Publisher('/timer_cam2_rec/memory/memory_line/line',LineArrayStamped , queue_size=1)
pub_image = rospy.Publisher('/timer_cam2_rec/memory/memory_line/debug_image', Image , queue_size=1)


sub_lines = message_filters.Subscriber('/timer_cam2_rec/memory/hough_lines/lines', LineArrayStamped)
sub_area = message_filters.Subscriber('/timer_cam2_rec/memory_edge', PoseArray)
sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)

sync = message_filters.ApproximateTimeSynchronizer([sub_lines,sub_area,sub_image], 10, 0.9)
sync.registerCallback(callback)
rospy.spin()
