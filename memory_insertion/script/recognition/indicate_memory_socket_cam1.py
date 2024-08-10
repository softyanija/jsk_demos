#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import message_filters
import numpy as np
import math
import time
from opencv_apps.msg import Line,LineArrayStamped,RotatedRectArrayStamped
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge


class MemorySocketCam1():

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
        self.bridge = CvBridge()
        self.pub_under = rospy.Publisher('/timer_cam1_rec/memory/memory_under', PoseArray, queue_size=1)
        self.pub_image = rospy.Publisher('/timer_cam1_rec/memory/memory_under/debug_image', Image, queue_size=1)
        self.subscribe()


    def subscribe(self):
        rospy.loginfo("hoge")
        sub_memory_rect = message_filters.Subscriber('/timer_cam1_rec/memory/general_contours/rectangles', RotatedRectArrayStamped)
        sub_socket_rect = message_filters.Subscriber('/timer_cam1_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam1/timer_cam_image/image_rect_color', Image)         
        self.subs = [sub_memory_rect, sub_socket_rect, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, memory_rect, socket_rect, image):
        self.sub_memory_rect = memory_rect
        self.sub_socket_rect = socket_rect
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header
        self.memory_under = PoseArray()
        # socket
        


    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("start indicate_memory_edge_cam1")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()

                if (not self.sub_memory_rect.rects == []):
                    size_max = 0.0
                    use_rect = None
                    for i,rect in enumerate(self.sub_memory_rect.rects):
                        size_buf = rect.size.width * rect.size.height
                        if size_buf > size_max and rect.center.y > 160:
                            size_max = size_buf
                            use_rect = i

                    if use_rect is not None:
                        self.memory_under_pose = Pose()
                        memory_len = max(self.sub_memory_rect.rects[use_rect].size.width, self.sub_memory_rect.rects[use_rect].size.height)
                        #memory_len = rec_msg.rects[use_rect].size.width
                        # under_x = int(rec_msg.rects[use_rect].center.x)
                        # under_y = int(rec_msg.rects[use_rect].center.y + memory_len/2)
            
                        # if (abs(self.sub_memory_rect.rects[use_rect].angle) < (self.sub_memory_rect.rects[use_rect].angle + 90)):
                        #     angle = self.sub_memory_rect.rects[use_rect].angle
                        # else:
                        #     angle = (self.sub_memory_rect.rects[use_rect].angle + 90)
                        angle = self.sub_memory_rect.rects[use_rect].angle
                        print(angle)
            
                        under_x = int(self.sub_memory_rect.rects[use_rect].center.x + (memory_len / 2)* math.sin(angle))
                        under_y = int(self.sub_memory_rect.rects[use_rect].center.y + (memory_len / 2)* math.cos(angle))
                        top_x = int(self.sub_memory_rect.rects[use_rect].center.x - (memory_len / 2)* math.sin(angle))
                        top_y = int(self.sub_memory_rect.rects[use_rect].center.y - (memory_len / 2)* math.cos(angle))
                        self.memory_under_pose.position.x = under_x
                        self.memory_under_pose.position.y = under_y
                        self.memory_under.header = self.header
                        self.memory_under.poses.append(self.memory_under_pose)

                        self.result_image = cv2.circle(self.result_image, (under_x,under_y), 3,(0,0,255),1,4,0)
                        # self.result_image = cv2.circle(self.result_image, (top_x,top_y), 5,(255,0,0),2,4,0)
                        self.result_image = cv2.line(self.result_image, (under_x,under_y),(top_x,top_y),(0,255,0),1)
                        self.pub_under.publish(self.memory_under)

                if (not self.sub_socket_rect.rects == []):
                    size_max = 0.0
                    use_rect = None
                    for i,rect in enumerate(self.sub_socket_rect.rects):
                        size_buf = rect.size.width * rect.size.height
                        if size_buf > size_max and rect.center.y > 160:
                            size_max = size_buf
                            use_rect = i

                    if use_rect is not None:
                        self.socket_pose = Pose()
                        #memory_len = max(self.sub_memory_rect.rects[use_rect].size.width, self.sub_memory_rect.rects[use_rect].size.height)
                        socket_center = self.sub_socket_rect.rects[use_rect].center
                        socket_size = self.sub_socket_rect.rects[use_rect].size
                        socket_angle = self.sub_socket_rect.rects[use_rect].angle
                        # socket_left_top = (int(socket_center.x - socket_size.width/2), int(socket_center.y - socket_size.height/2))
                        # socket_right_bottom = (int(socket_center.x + socket_size.width/2), int(socket_center.y + socket_size.height/2))
                        socket_right_bottom = (int(socket_center.x - socket_size.width/2 * math.cos(math.radians(socket_angle)) - socket_size.height/2 * math.sin(math.radians(socket_angle))),
                                           int(socket_center.y - socket_size.width/2 * math.sin(math.radians(socket_angle)) - socket_size.height/2 * math.cos(math.radians(socket_angle))))
                        socket_left_top = (int(socket_center.x + socket_size.width/2 * math.cos(math.radians(socket_angle)) + socket_size.height/2 * math.sin(math.radians(socket_angle))),
                                               int(socket_center.y + socket_size.width/2 * math.sin(math.radians(socket_angle)) + socket_size.height/2 * math.cos(math.radians(socket_angle))))
                        socket_target_point = (int(socket_center.x + socket_size.width/2 * math.cos(math.radians(socket_angle))),
                                               int(socket_center.y + socket_size.width/2 * math.sin(math.radians(socket_angle))))
                        socket_target_line_end = (int(socket_center.x + socket_size.width * math.cos(math.radians(socket_angle))),
                                               int(socket_center.y + socket_size.width * math.sin(math.radians(socket_angle))))

                        self.result_image = cv2.rectangle(self.result_image, socket_left_top, socket_right_bottom, (0,255,0),3)
                        self.result_image = cv2.circle(self.result_image, socket_target_point, 3,(255,0,0),3,4,0)
                        self.result_image = cv2.line(self.result_image, socket_target_point, socket_target_line_end, (0,0,255), 4)                                       
                        
                self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
                self.pub_image.publish(self.result_image)

            else:
                rospy.logwarn("didn't recieve image")


if __name__ == "__main__":
    rospy.init_node('indicate_memory_socket_cam1')

    memory_socket_cam1 = MemorySocketCam1()

    memory_socket_cam1.run()
