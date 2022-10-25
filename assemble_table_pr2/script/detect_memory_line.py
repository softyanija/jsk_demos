#!/usr/bin/env python

import cv2
import cv_bridge
import message_filters
import numpy as np
import rospy
import time
from opencv_apps.msg import LineArrayStamped
from geometry_msgs.msg import PoseArray

class DetectLines():

    def __init__(self):

        self.subs = []
        self.line_pub = rospy.Publisher('/timer_cam2_rec/memory/memory_line',LineArrayStamped , queue_size=1)
        rospy.loginfo("hoge")

        
    def callback(self, lines, area): ##how to use data
        #for debug
        #cv_img = bridge.imgmsg_to_cv2()
        pospy.loginfo("callback")
        memory_line = LineArrayStamped()
        self.line_pub(memory_line)        

    def subcribe(self):
        rospy.loginfo("poyo")
        sub_lines = message_filters.Subscriber('/timer_cam2_rec/memory/hough_lines/lines', LineArrayStamped)
        sub_area = message_filters.Subscriber('/timer_cam2_rec/memory_edge', PoseArray)
        warn_no_remap('/timer_cam2_rec/memory/hough_lines/lines', '/timer_cam2_rec/memory_edge')

        subs = [sub_lines, sub_area]

        if self.approximate_sync:
            slop = 0.1
            sync = message_filters.TimeSyncrhonizer(subs, queue_size = self.queue_size)
            sync.registerCallback(self.callback)

        self.subs = subs

    def line_publish(self,data):
        self.line_pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('detect_lines')
    dl = DetectLines()
    rospy.spin()

