#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

dictionary_name = cv2.aruco.DICT_4X4_50
dictionary = cv2.aruco.getPredefinedDictionary(dictionary_name)

class ArucoHandPos():

    def __init__(self):
        self.bridge = CvBridge()
        self.result_image = None
        self.hand_pos = None
        self.sub_image = rospy.Subscriber("/timer_cam2/timer_cam_image/image_rect_color", Image, self.callback)
        self.pub_hand_pos = rospy.Publisher("/timer_cam2_rec/hand_pos/position", PoseArray, queue_size=3)
        self.pub_image = rospy.Publisher("/timer_cam2_rec/hand_pos/debug_image", Image, queue_size=3)


    def callback(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")

    def run(self):
        self.hand_pos = PoseArray()
        self.hand_pos.header = image.header
        self.result_image = self.sub_image.copy()

        corners, ids, rejectImgPoints = cv2.aruco.detectMarkers(self.result_image, dictionary)

        if ids is not None:
            v = np.mean(corners[0][0],axis=0)
            print (int(v[0]), int(v[1]))

            pose = Pose()
            pose.position.x = int(v[0])
            pose.position.y = int(v[1])
            pose.position.z= 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            self.hand_pos.poses.append(pose)

            self.result_image = cv2.circle(self.result_image, (int(v[0]), int(v[1]) + 40), 5,(0,0,255),1,4,0)

        self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "bgr8")
        self.pup_edge.publish(self.hand_pos)
        self.pub_image.publish(self.result_image)


if __name__ == '__main__':
    rospy.init_node('aruco_hand_pos')
    aruco_hand_pos = ArucoHandPos()
    

