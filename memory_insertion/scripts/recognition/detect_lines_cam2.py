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


class DetectLinesCam2():

    def __init__(self):
        self.header = None
        self.lines_msg = None
        self.sub_image = None
        self.clip_top_left = (0, 130)
        self.clip_bottom_right = (200, 200)        
        self.bridge = CvBridge()
        self.line = Line()
        self.lines_msg = LineArrayStamped()
        
        self.pub_equalized_image = rospy.Publisher('/timer_cam2_rec/socket/equalized_image', Image, queue_size=1)
        self.pub_contour_image = rospy.Publisher('/timer_cam2_rec/socket/contour_image', Image, queue_size=1)
        self.pub_result_image = rospy.Publisher('/timer_cam2_rec/socket/detected_lines_image', Image, queue_size=1)
        self.pub_lines = rospy.Publisher("/timer_cam2_rec/socket/detected_lines", LineArrayStamped, queue_size=2)
        self.subscribe_image()


    def subscribe_image(self):
            sub_image = rospy.Subscriber("/timer_cam2/timer_cam_image/image_rect_color", Image, self.image_callback)


    def image_callback(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header


    def draw_line(self, img, theta, rho):
        h, w = img.shape[:2]
        if np.isclose(np.sin(theta), 0):
            x1, y1 = rho, 0
            x2, y2 = rho, h
        else:
            calc_y = lambda x: rho / np.sin(theta) - x * np.cos(theta) / np.sin(theta)
            x1, y1 = 0, calc_y(0)
            x2, y2 = w, calc_y(w)
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return (x1, y1), (x2, y2)


    def run(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            self.line = Line()
            self.lines_msg = LineArrayStamped()
            
            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                self.lines_msg.header = self.header
        
                gray = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                equ = cv2.equalizeHist(gray)
                equalized_image_wide = np.hstack((gray,equ))
                equalized_image = equalized_image_wide[:, equalized_image_wide.shape[1]//2:equalized_image_wide.shape[1]]
               
                image_contour_only = cv2.Canny(equalized_image, 100, 250, L2gradient=True)

                mask = np.zeros_like(image_contour_only)
                cv2.rectangle(mask, self.clip_top_left, self.clip_bottom_right, (255,255,255), thickness=cv2.FILLED)
                cliped_contour = cv2.bitwise_and(image_contour_only, mask)
                detected_lines = cv2.HoughLines(cliped_contour, rho=1, theta=np.pi/180, threshold=100)

                
                if not detected_lines is None:
                    for line in detected_lines:
                        rho, theta = line[0]
                        print(self.result_image.shape[:2])
                        pt1, pt2 = self.draw_line(self.result_image, theta, rho)
                        self.line.pt1.x = pt1[0]
                        self.line.pt1.y = pt1[1]
                        self.line.pt2.x = pt2[0]
                        self.line.pt2.y = pt2[1]
                        self.lines_msg.lines.append(self.line)

                
                # rospy.loginfo(detected_lines)
                
                self.equalized_image = self.bridge.cv2_to_imgmsg(equalized_image, "mono8")
                self.contour_image = self.bridge.cv2_to_imgmsg(cliped_contour, "8UC1")
                self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
                
                self.pub_equalized_image.publish(self.equalized_image)
                self.pub_contour_image.publish(self.contour_image)
                self.pub_result_image.publish(self.result_image)
                self.pub_lines.publish(self.lines_msg)

            else:
                rospy.logwarn("didn't recieve image")


if __name__ == "__main__":
    rospy.init_node('detect_lines_cam2')

    detect_lines_cam2 = DetectLinesCam2()

    detect_lines_cam2.run()
                    
            
