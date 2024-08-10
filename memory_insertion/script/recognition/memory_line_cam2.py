#!/usr/bin/env python

import cv2
import cv_bridge
import message_filters
import numpy as np
import rospy
import time
from opencv_apps.msg import LineArrayStamped
from geometry_msgs.msg import PoseArray

class MemoryLineCam2():

    def __init__(self):
        self.subs = []
        self.header = None
        self.lines_msg = None
        self.sub_image = None
        self.sub_memory_rect = None
        self.sub_socket_rect = None
        self.memory_under = None
        self.result_image = None
        self.memory_under = None
        self.memory_angle = None
        self.socket_angle = None
        self.bridge = CvBridge()
        self.pub_memory_line = rospy.Publisher('/timer_cam2_rec/memory/memory_line/line', LineArrayStamped, queue_size=1)
        self.pub_image = rospy.Publisher('/timer_cam2_rec/memory/memory/memory_line/debug_image', Image, queue_size=1)
        self.subscribe()


    def subcribe(self):
        sub_lines = message_filters.Subscriber('/timer_cam2_rec/memory/hough_lines/lines', LineArrayStamped)
        sub_area = message_filters.Subscriber('/timer_cam2_rec/memory_edge', PoseArray)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)
        subs = [sub_lines, sub_area, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, lines, area, image): ##how to use data
        #for debug
        pospy.loginfo("callback")
        self.sub_memory_lines = lines.lines
        self.sub_memory_area = area.poses
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header
        # memory_line = LineArrayStamped()

    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("start memory_line_cam2")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                
                if ((not self.sub_memory_lines == []) and (not self.sub_memory_area == [])):
        
                    l1 = (self.sub_memory_area[1].position.x - self.sub_memory_area[0].position.x)**2 + (self.sub_memory_area[1].position.y - self.sub_memory_area[0].position.y)**2
                    l2 = (self.sub_memory_area[1].position.x - self.sub_memory_area[2].position.x)**2 + (self.sub_memory_area[1].position.y - self.sub_memory_area[2].position.y)**2

                    if l1 > l2:
                        t = (self.sub_memory_area[1].position.y - self.sub_memory_area[0].position.y) / max(1, (self.sub_memory_area[1].position.x - self.sub_memory_area[0].position.x))
                    else:
                        t = (self.sub_memory_area[1].position.y - self.sub_memory_area[2].position.y) / max(1, (self.sub_memory_area[1].position.x - self.sub_memory_area[2].position.x))

                    ref_deg = math.atan(t)
                    use_line = 0
        
                    for i,line in enumerate(self.sub_memory_lines.lines):
                        buf = (line.pt1.y - line.pt2.y) / max(1, (line.pt1.x - line.pt2.x))
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

                    memory_line = Line()
                    
                    if (abs(diff_deg_min) < 0.2):
                        line = lines_msg.lines[use_line]
                        memory_line.lines.append(line)
                        self.result_image = cv2.line(self.result_image, line_start, line_end, (0,255,0), 3)

                    self.result_image = bridge.cv2_to_imgmsg(self.result_image, "bgr8")
    
                pub_line.publish(memory_line)
                pub_image.publish(self.result_image)

            else:
                rospy.logwarn("didn't recieve image")


if __name__ == '__main__':
    rospy.init_node('memory_line_cam2')
    memory_line_cam2 = MemoryLineCam2()
    rospy.spin()

