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


class SocketCam2():

    def __init__(self):
        self.subs = []
        self.header = None
        self.lines_msg = None
        self.sub_image = None
        self.sub_socket_rect = None
        self.result_image = None
        self.socket_angle = None
        self.offset_x = 10
        self.offset_y = 38
        self.bridge = CvBridge()
        self.pub_socket_target_point_b = rospy.Publisher('/timer_cam2_rec/socket/target_point_b', PoseArray, queue_size=1)
        self.pub_image = rospy.Publisher('/timer_cam2_rec/socket/debug_image', Image, queue_size=1)
        self.subscribe()


    def subscribe(self):
        sub_socket_rect = message_filters.Subscriber('/timer_cam2_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)         
        self.subs = [sub_socket_rect, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, socket_rect, image):
        self.sub_socket_rect = socket_rect
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header
        # socket


    def run(self):
        rate = rospy.Rate(2)
        rospy.loginfo("start socket_cam2")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                self.socket_angle = None

                if (not self.sub_socket_rect.rects == []):
                    size_max = 0.0
                    use_rect = None
                    for i,rect in enumerate(self.sub_socket_rect.rects):
                        size_buf = rect.size.width * rect.size.height
                        if size_buf > size_max and rect.center.y > 100:
                            size_max = size_buf
                            use_rect = i

                    if use_rect is not None:
                        self.socket_target_point_b = PoseArray()
                        self.socket_target_point_b.header = self.header
                        self.socket_pose = Pose()
                        #memory_len = max(self.sub_memory_rect.rects[use_rect].size.width, self.sub_memory_rect.rects[use_rect].size.height)
                        socket_center = self.sub_socket_rect.rects[use_rect].center
                        socket_size = self.sub_socket_rect.rects[use_rect].size
                        socket_angle = self.sub_socket_rect.rects[use_rect].angle

                        self.socket_angle = socket_angle

                        if self.sub_socket_rect.rects[use_rect].size.width > self.sub_socket_rect.rects[use_rect].size.height:
                            self.socket_angle = socket_angle + 90
                            socket_right_bottom = (int(socket_center.x - socket_size.width/2 * math.sin(math.radians(self.socket_angle)) + socket_size.height/2 * math.cos(math.radians(self.socket_angle))),
                                                   int(socket_center.y + socket_size.width/2 * math.cos(math.radians(self.socket_angle)) + socket_size.height/2 * math.cos(math.radians(self.socket_angle))))
                            socket_left_top = (int(socket_center.x + socket_size.width/2 * math.sin(math.radians(self.socket_angle)) - socket_size.height/2 * math.cos(math.radians(self.socket_angle))),
                                               int(socket_center.y - socket_size.width/2 * math.cos(math.radians(self.socket_angle)) - socket_size.height/2 * math.cos(math.radians(self.socket_angle))))
                            
                        else:
                            self.socket_angle = socket_angle
                            socket_right_bottom = (int(socket_center.x - socket_size.height/2 * math.sin(math.radians(self.socket_angle)) + socket_size.width/2 * math.cos(math.radians(self.socket_angle))),
                                                   int(socket_center.y + socket_size.height/2 * math.cos(math.radians(self.socket_angle)) + socket_size.width/2 * math.cos(math.radians(self.socket_angle))))
                            socket_left_top = (int(socket_center.x + socket_size.height/2 * math.sin(math.radians(self.socket_angle)) - socket_size.width/2 * math.cos(math.radians(self.socket_angle))),
                                               int(socket_center.y - socket_size.height/2 * math.cos(math.radians(self.socket_angle)) - socket_size.width/2 * math.cos(math.radians(self.socket_angle))))

                        offset_vector = (- self.offset_x * math.cos(math.radians(self.socket_angle)) + self.offset_y * math.sin(math.radians(self.socket_angle)),
                                            + self.offset_x * math.sin(math.radians(self.socket_angle)) - self.offset_y * math.cos(math.radians(self.socket_angle)))
                        target_vector_scolor = 15
                        socket_target_point = (int(socket_center.x + offset_vector[0]),
                                               int(socket_center.y + offset_vector[1]))
                        socket_target_line_end = (int(socket_target_point[0] - target_vector_scolor * math.sin(math.radians(self.socket_angle))),
                                                  int(socket_target_point[1] - target_vector_scolor * math.cos(math.radians(self.socket_angle))))


                        self.socket_pose.position.x = socket_left_top[0]
                        self.socket_pose.position.y = socket_left_top[1]
                        self.socket_pose.position.z = 0
                        self.socket_pose.orientation.x = 0
                        self.socket_pose.orientation.y = 0
                        self.socket_pose.orientation.z = 0
                        self.socket_pose.orientation.w = 1
                        self.socket_target_point_b.poses.append(self.socket_pose)

                        self.result_image = cv2.rectangle(self.result_image, socket_left_top, socket_right_bottom, (0,255,0),3)
                        #self.result_image = cv2.circle(self.result_image, socket_left_top, 3,(255,0,0),3,4,0)
                        self.result_image = cv2.line(self.result_image, socket_target_point, socket_target_line_end, (0,255,0), 3)
                        self.result_image = cv2.circle(self.result_image, socket_target_point, 2,(255,0,0),2,2,0)

                self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
                self.pub_image.publish(self.result_image)
                self.pub_socket_target_point_b.publish(self.socket_target_point_b)

            else:
                rospy.logwarn("didn't recieve image")


if __name__ == "__main__":
    rospy.init_node('socket_cam2')

    socket_cam2 = SocketCam2()

    socket_cam2.run()
