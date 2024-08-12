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



class SocketLineCam2():
    def __init__(self):
        self.subs = []
        self.header = None
        self.lines_msg = None
        self.sub_image = None
        self.socket_line = None
        self.bridge = CvBridge()
        self.pub_line = rospy.Publisher('/timer_cam2_rec/socket/socket_line/line',LineArrayStamped , queue_size=2)
        self.pub_image = rospy.Publisher('/timer_cam2_rec/socket/socket_line/debug_image', Image , queue_size=2)
         

    def subscribe(self):
        sub_lines = message_filters.Subscriber("/timer_cam2_rec/socket/hough_lines/lines", LineArrayStamped)
        sub_image = message_filters.Subscriber("/timer_cam2/timer_cam_image/image_rect_color", Image)
        self.subs = [sub_lines, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, lines, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = lines.header


    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("start socket_line")
             
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                 rospy.logwarn("cought {}".format(e))
                 pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                
                if (not self.lines_msg.lines == []):
                    use_line = None
                    center_y_max = 0
                    for i,line in enumerate(self.lines_msg.lines):
                        center_y = (line.pt1.y + line.pt2.y)/2
                        if center_y > center_y_max:
                            center_y_max = center_y
                            use_line = i

                    if use_line is not None:
                        if center_y_max > 320 and (2* abs(lines_msg.lines[use_line].pt1.y - lines_msg.lines[use_line].pt2.y))< abs(lines_msg.lines[use_line].pt1.x - lines_msg.lines[use_line].pt2.x) :
                            line = lines_msg.lines[use_line]
                            socket_line.lines.append(line)
                            line_start = (int(line.pt1.x), int(line.pt1.y))
                            line_end = (int(line.pt2.x), int(line.pt2.y))
                            self.result_image = cv2.line(self.result_image, line_start, line_end, (0,255,0), 3)
                            self.pub_line.publish(socket_line)

                self.pub_image.publish(self.bridge.cv2_to_imgmsg(self.result_image, "bgr8"))

            else:
                rospy.logwarn("didn't recieve lines")

if __name__ == "__main__":

    rospy.init_node('socket_line')
    socket_line = SocketLine()
    socket_line.run()
