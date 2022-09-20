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

rospy.init_node('detect_lines')

def callback(lines_msg, area_msg): ##how to use data
    #for debug
    #cv_img = bridge.imgmsg_to_cv2()
    rospy.loginfo("callback")
    memory_line = LineArrayStamped()
    memory_line.header = lines_msg.header

    if ((not lines_msg.lines == []) and (not area_msg.poses == [])):
        rospy.loginfo("process")
        
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

            if diff_deg_min > diff_deg:
                diff_deg_min = diff_deg
                use_line = i
         
        line = Line()
        line = lines_msg.lines[use_line]
        memory_line.lines.append(line)
    
    pub_line.publish(memory_line)


pub_line = rospy.Publisher('/timer_cam2_rec/memory/memory_line',LineArrayStamped , queue_size=1)
sub_lines = message_filters.Subscriber('/timer_cam2_rec/memory/hough_lines/lines', LineArrayStamped)
sub_area = message_filters.Subscriber('/timer_cam2_rec/memory_edge', PoseArray)

sync = message_filters.ApproximateTimeSynchronizer([sub_lines,sub_area], 10, 0.9)
sync.registerCallback(callback)
rospy.spin()

#     def __init__(self):

#         self.subs = []
#         self.pub = rospy.Publisher('/timer_cam2_rec/memory/memory_line',LineArrayStamped , queue_size=1)
#         rospy.loginfo("hoge")

        
#    def callback(self, lines, area): ##how to use data
#         #for debug
#         #cv_img = bridge.imgmsg_to_cv2()
#         memory_line = LineArrayStamped()
#         self.pub(memory_line)

        

#     def subcribe(self):
#         rospy.loginfo("poyo")
        
#         warn_no_remap('/timer_cam2_rec/memory/hough_lines/lines', '/timer_cam2_rec/memory_edge')

#         subs = [sub_lines, sub_area]

#         if self.approximate_sync:
#             slop = 0.1
            

#         self.subs = subs

# if __name__ == '__main__':
#     rospy.init_node('detect_lines')
#     dl = DetectLines()
#     rospy.spin()

