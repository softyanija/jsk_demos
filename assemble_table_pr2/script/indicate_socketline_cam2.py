#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import message_filters
import numpy as np
import math
import time
import pdb
from opencv_apps.msg import Line,LineArray,LineArrayStamped,RotatedRectArrayStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class SocketLine():

    def __init__(self):
        self.sub_lines = None
        self.sub_image = None
        self.debug_image = None
        self.socket_line = None
        self.socket_line_candidate = None
        self.header = None
        self.bridge = CvBridge()
        
        self.pub_line = rospy.Publisher('/timer_cam2_rec/socket/socket_line/line',LineArrayStamped , queue_size=1)
        self.pub_debug_image = rospy.Publisher('/timer_cam2_rec/socket/socket_line/debug_image', Image , queue_size=1)

        self.multi_subscribe()


    def multi_subscribe(self):
        sub_lines = message_filters.Subscriber('/timer_cam2_rec/socket/hough_lines/lines', LineArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)
        self.subs = [sub_lines, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.multi_callback)


    def multi_callback(self, lines, image):
        self.sub_lines = lines 
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = image.header


    def run(self):
        rate = rospy.Rate(5)
        self.socket_line = LineArrayStamped()
        
        rospy.loginfo("start to recognition socket line")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
            
            if self.sub_image is not None:
                self.pub_image = self.sub_image.copy()
                
                if (not self.sub_lines == []):
                    use_line = 0
                    center_y_max = 0
                    self.socket_line_candidate = LineArray()
                    
                    for i,line in enumerate(self.sub_lines.lines):
                        center_y = (line.pt1.y + line.pt2.y)/2

                        if center_y > 170 and center_y < 210 and (2* abs(line.pt1.y - line.pt2.y))< abs(line.pt1.x - line.pt2.x):
                            # rospy.loginfo("selected line")
                            # rospy.loginfo(center_y)
                            
                            self.socket_line_candidate.lines.append(line)
                            line_start = (int(line.pt1.x), int(line.pt1.y))
                            line_end = (int(line.pt2.x), int(line.pt2.y))
                            self.pub_image = cv2.line(self.pub_image, line_start, line_end, (0,255,0), 3)

                    if (not self.socket_line_candidate.lines == []):
                        for j,line in enumerate(self.socket_line_candidate.lines):
                            max_length = 0
                            use_line_id = 0
                            length = (line.pt1.x - line.pt2.x)**2 + (line.pt1.y - line.pt2.y)**2
                            if length > max_length:
                                max_length = length
                                use_line_id = j

                        self.socket_line.lines.append(self.socket_line_candidate.lines[use_line_id])
                        socket_line_start = (int(self.socket_line_candidate.lines[use_line_id].pt1.x), int(self.socket_line_candidate.lines[use_line_id].pt1.y))
                        socket_line_end = (int(self.socket_line_candidate.lines[use_line_id].pt2.x), int(self.socket_line_candidate.lines[use_line_id].pt2.y))
                        self.pub_image = cv2.line(self.pub_image, line_start, line_end, (0,0,255), 3)
                
                self.socket_line.header = self.header
                self.pub_line.publish(self.socket_line)
                self.pub_debug_image.publish(self.bridge.cv2_to_imgmsg(self.pub_image, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("socket_line")
    socket_line = SocketLine()
    socket_line.run()
    
