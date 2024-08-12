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

rospy.init_node('socket_line')

def callback(lines_msg,image_msg):

    bridge = CvBridge()
    socket_line = LineArrayStamped()
    socket_line.header = lines_msg.header

    if (not lines_msg.lines == []):
        use_line = 0
        center_y_max = 0
        for i,line in enumerate(lines_msg.lines):
            center_y = (line.pt1.y + line.pt2.y)/2
            if center_y > center_y_max:
                center_y_max = center_y
                use_line = i

        line = Line()
        image_msg = bridge.imgmsg_to_cv2(image_msg, "bgr8")

        if center_y_max > 150 and (2* abs(lines_msg.lines[use_line].pt1.y - lines_msg.lines[use_line].pt2.y))< abs(lines_msg.lines[use_line].pt1.x - lines_msg.lines[use_line].pt2.x) :
            line = lines_msg.lines[use_line]
            socket_line.lines.append(line)
            line_start = (int(line.pt1.x), int(line.pt1.y))
            line_end = (int(line.pt2.x), int(line.pt2.y))
            image_msg = cv2.line(image_msg, line_start, line_end, (0,255,0), 2)
        image_msg = bridge.cv2_to_imgmsg(image_msg, "bgr8")
        pub_line.publish(socket_line)
        pub_image.publish(image_msg)


pub_line = rospy.Publisher('/timer_cam2_rec/socket/socket_line/line',LineArrayStamped , queue_size=1)
pub_image = rospy.Publisher('/timer_cam2_rec/socket/socket_line/debug_image', Image , queue_size=1)


sub_lines = message_filters.Subscriber('/timer_cam2_rec/socket/hough_lines/lines', LineArrayStamped)
sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)

sync = message_filters.ApproximateTimeSynchronizer([sub_lines,sub_image], 10, 0.5)
sync.registerCallback(callback)
rospy.spin()
