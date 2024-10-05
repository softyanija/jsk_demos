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


class MemorySocketCam2():

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
        self.pub_guide_point_b = rospy.Publisher("/timer_cam2_rec/memory_edge/guide_point_b", PoseArray, queue_size=3)
        self.pub_memory_edge = rospy.Publisher("/timer_cam2_rec/memory_edge/memory_edge", PoseArray, queue_size=3)
        self.pub_socket_target_point_b = rospy.Publisher('/timer_cam2_rec/socket/target_point_b', PoseArray, queue_size=1)
        self.pub_image = rospy.Publisher('/timer_cam2_rec/memory_socket/debug_image', Image, queue_size=1)
        self.subscribe()


    def subscribe(self):
        sub_memory_rect = message_filters.Subscriber('/timer_cam2_rec/memory/general_contours/rectangles', RotatedRectArrayStamped)
        sub_memory_hsv_image = message_filters.Subscriber("/timer_cam2_rec/memory/hsv_color_filter/image", Image)
        sub_socket_rect = message_filters.Subscriber('/timer_cam2_rec/socket/general_contours/rectangles', RotatedRectArrayStamped)
        sub_image = message_filters.Subscriber('/timer_cam2/timer_cam_image/image_rect_color', Image)         
        self.subs = [sub_memory_rect, sub_memory_hsv_image, sub_socket_rect, sub_image]
        sync = message_filters.ApproximateTimeSynchronizer(fs=self.subs, queue_size=5, slop=1)
        sync.registerCallback(self.callback)


    def callback(self, memory_rect, memory_hsv_image, socket_rect, image):
        self.sub_memory_rect = memory_rect
        self.sub_memory_hsv_image = self.bridge.imgmsg_to_cv2(memory_hsv_image, "rgb8")
        self.sub_socket_rect = socket_rect
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header
        # socket

    def oneshot(self):
        if (self.sub_image is not None):
            self.result_image = self.sub_image.copy()

            if (not self.sub_memory_rect.rects == []):
                self.memory_edge = PoseArray()
                self.memory_edge.header = self.header
                self.memory_guide_point_b = PoseArray()
                self.memory_guide_point_b.header = self.header
                img_gray = cv2.cvtColor(self.sub_memory_hsv_image, cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(img_gray,127,255,0)
                contours,hierarchy = cv2.findContours(thresh, 1, 2)
        
                if (not contours == []):
                    cnt = max(contours, key=lambda x: cv2.contourArea(x))
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    buf = box.tolist()

                    max_sum_xy = 0
                    use_index = 0
                    for i in range(4):
                        pose = Pose()
                        pose.position.x = box[i][0]
                        pose.position.y = box[i][1]
                        pose.position.z= 0
                        pose.orientation.x = 0
                        pose.orientation.y = 0
                        pose.orientation.z = 0
                        pose.orientation.w = 1

                        #all edge

                        self.result_image = cv2.circle(self.result_image, (pose.position.x, pose.position.y), 2, (0,0,255), 2,2,0)
                        self.memory_edge.poses.append(pose)

                        if (box[i][0] + box[i][1]) > max_sum_xy:
                            max_sum_xy = box[i][0] + box[i][1]
                            use_index = i

                    pose.position.x = box[use_index][0]
                    pose.position.y = box[use_index][1]
                    pose.position.z= 0
                    pose.orientation.x = 0
                    pose.orientation.y = 0
                    pose.orientation.z = 0
                    pose.orientation.w = 1

                    self.memory_guide_point_b.poses.append(pose)
                    rospy.loginfo("memory edge x: " + str(self.memory_edge.poses[use_index].position.x) + " y: " + str(self.memory_edge.poses[use_index].position.y))
                    self.result_image = cv2.circle(self.result_image, (self.memory_edge.poses[use_index].position.x, self.memory_edge.poses[use_index].position.y), 3, (255,0,0), -1,3,0)

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
                    self.socket_center = self.sub_socket_rect.rects[use_rect].center
                    self.socket_size = self.sub_socket_rect.rects[use_rect].size
                    socket_angle = self.sub_socket_rect.rects[use_rect].angle

                    self.socket_angle = socket_angle

                    if self.sub_socket_rect.rects[use_rect].size.width > self.sub_socket_rect.rects[use_rect].size.height:
                        self.socket_angle = socket_angle + 90
                        socket_right_bottom = (int(self.socket_center.x - self.socket_size.width/2 * math.sin(math.radians(self.socket_angle)) + self.socket_size.height/2 * math.cos(math.radians(self.socket_angle))),
                                               int(self.socket_center.y + self.socket_size.width/2 * math.cos(math.radians(self.socket_angle)) + self.socket_size.height/2 * math.cos(math.radians(self.socket_angle))))
                        socket_left_top = (int(self.socket_center.x + self.socket_size.width/2 * math.sin(math.radians(self.socket_angle)) - self.socket_size.height/2 * math.cos(math.radians(self.socket_angle))),
                                           int(self.socket_center.y - self.socket_size.width/2 * math.cos(math.radians(self.socket_angle)) - self.socket_size.height/2 * math.cos(math.radians(self.socket_angle))))
                            
                    else:
                        self.socket_angle = socket_angle
                        socket_right_bottom = (int(self.socket_center.x - self.socket_size.height/2 * math.sin(math.radians(self.socket_angle)) + self.socket_size.width/2 * math.cos(math.radians(self.socket_angle))),
                                               int(self.socket_center.y + self.socket_size.height/2 * math.cos(math.radians(self.socket_angle)) + self.socket_size.width/2 * math.cos(math.radians(self.socket_angle))))
                        socket_left_top = (int(self.socket_center.x + self.socket_size.height/2 * math.sin(math.radians(self.socket_angle)) - self.socket_size.width/2 * math.cos(math.radians(self.socket_angle))),
                                           int(self.socket_center.y - self.socket_size.height/2 * math.cos(math.radians(self.socket_angle)) - self.socket_size.width/2 * math.cos(math.radians(self.socket_angle))))

                    offset_vector = (- self.offset_x * math.cos(math.radians(self.socket_angle)) + self.offset_y * math.sin(math.radians(self.socket_angle)),
                                     + self.offset_x * math.sin(math.radians(self.socket_angle)) - self.offset_y * math.cos(math.radians(self.socket_angle)))
                    target_vector_scolor = 15
                    self.socket_target_point = (int(self.socket_center.x + offset_vector[0]),
                                                int(self.socket_center.y + offset_vector[1]))
                    socket_target_line_end = (int(self.socket_target_point[0] - target_vector_scolor * math.sin(math.radians(self.socket_angle))),
                                              int(self.socket_target_point[1] - target_vector_scolor * math.cos(math.radians(self.socket_angle))))

                    self.socket_pose.position.x = socket_left_top[0]
                    self.socket_pose.position.y = socket_left_top[1]
                    self.socket_pose.position.z = 0
                    self.socket_pose.orientation.x = 0
                    self.socket_pose.orientation.y = 0
                    self.socket_pose.orientation.z = 0
                    self.socket_pose.orientation.w = 1
                    self.socket_target_point_b.poses.append(self.socket_pose)

                    self.result_image = cv2.rectangle(self.result_image, socket_left_top, socket_right_bottom, (0,255,0),2)
                    self.result_image = cv2.line(self.result_image, self.socket_target_point, socket_target_line_end, (0,255,0), 2)
                    self.result_image = cv2.circle(self.result_image, self.socket_target_point, 3,(255,0,0),-1,2,0)
                     

            self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
            self.pub_image.publish(self.result_image)
            self.pub_memory_edge.publish(self.memory_edge)
            self.pub_guide_point_b.publish(self.memory_guide_point_b)
            self.pub_socket_target_point_b.publish(self.socket_target_point_b)

        else:
            rospy.logwarn("didn't recieve image")


    def loop(self):
        rate = rospy.Rate(2)
        rospy.loginfo("start socket_cam2")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            self.oneshot()


if __name__ == "__main__":
    rospy.init_node('memory_socket_cam2')

    memory_socket_cam2 = MemorySocketCam2()

    memory_socket_cam2.loop()
