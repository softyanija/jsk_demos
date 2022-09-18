#!/usr/bin/env python

import cv2
import cv_bridge
import message_filter
import numpy as np
import rospy
from opencv_apps import LineArrayStamped
from geometry_msgs import PoseArray

class DetectLines():

    def __init__(self):

        self.subs = []
        self.pub = self.advertise('output', , queue_size=1)

        
    def callback(self, lines, area): ##how to use data
        #for debug
        #cv_img = bridge.imgmsg_to_cv2()

        self.pub(memory_line)

        

    def subcribe(self):
        sub_lines = message_filter.Subscribe('/timer_cam2_rec/memory/hough_lines/lines', LineArrayStamped)
        sub_area = message_filter.Subscribe('/timer_cam2_rec/memory_edge', PoseArray)
        warn_no_remap('/timer_cam2_rec/memory/hough_lines/lines', '/timer_cam2_rec/memory_edge')

        subs = [sub_lines, sub_area]

        if self.approximate_sync:
            slop = 0.1
            sync = message_filters.TimeSyncrhonizer(subs, queue_size = self.queue_size)
            sysnc.registerCallback(self.callback)

        self.subs = subs

if __name__ == '__main__':
    rospy.init_node('detet_line')
    dl = DetectLines()
    rospy.spin()
