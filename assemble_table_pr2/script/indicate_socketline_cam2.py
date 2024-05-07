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

class SocketLine():

    def __init__(self):
        self.sub_lines = None
        self.sub_image = None
        self.debug_image = None
        self.socket_line = None
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
        rospy.loginfo("multi_callback")


    def run(self):
        rate = rospy.Rate(5)
        
        rospy.loginfo("start to recognition socket line")
        
        while not rospy.is_shutdown():
            rospy.loginfo("loop")
            try:
                rospy.loginfo("1")
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            rospy.loginfo("hoge")
            
            if self.sub_image is not None:
                if (not self.sub_lines == []):
                    use_line = 0
                    center_y_max = 0
                    for i,line in enumerate(self.sub_lines):
                        center_y = (line.pt1.y + line.pt2.y)/2
                        if center_y > center_y_max:
                            center_y_max = center_y
                            use_line = i

                    line = Line()
                    self.pub_img = bridge.imsmsg_to_cv2(self.sub_img, "bgr8")

                    if center_y_max > 150 and (2* abs(self.sub_lines.lines[use_line].pt1.y - self.sub_lines.lines[use_line].pt2.y))< abs(self.sub_lines.lines[use_line].pt1.x - self.sub_lines.lines[use_line].pt2.x) :
                        line = sub_lines.lines[use_line]
                        self.socket_line.lines.append(line)
                        line_start = (int(line.pt1.x), int(line.pt1.y))
                        line_end = (int(line.pt2.x), int(line.pt2.y))
                        self.pub_image = cv2.line(self.pub_image, line_start, line_end, (0,255,0), 2)


                self.pub_image = bridge.cv2_to_imgmsg(image_msg, "bgr8")
                self.pub_line.publish(self.socket_line)
                self.pub_debug_image.publish(self.pub_image)
                print("huga")

if __name__ == "__main__":
    rospy.init_node("socket_line")
    socket_line = SocketLine()
    socket_line.run()
    
