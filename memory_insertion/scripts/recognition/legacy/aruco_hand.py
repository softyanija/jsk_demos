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

class hand_pos():
    def __init__(self):
        self.sub = rospy.Subscriber("/timer_cam2/timer_cam_image/image_rect_color", Image, self.callback)
        self.edge_pub = rospy.Publisher("/timer_cam2_rec/hand_pos", PoseArray, queue_size=3)
        self.img_pub = rospy.Publisher("/timer_cam2_rec/hand_pos/debug_image", Image, queue_size=3)
        
    def callback(self, data):
        ps = PoseArray()
        ps.header = data.header
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        corners, ids, rejectImgPoints = cv2.aruco.detectMarkers(img, dictionary)

        if ids is not None:
            v = np.mean(corners[0][0],axis=0)
            print int(v[0]), int(v[1])

            pose = Pose()
            pose.position.x = int(v[0])
            pose.position.y = int(v[1])
            pose.position.z= 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            ps.poses.append(pose)

            img = cv2.circle(img, (int(v[0]),int(v[1]) + 20), 5,(0,0,255),1,4,0)

        img = bridge.cv2_to_imgmsg(img, "bgr8")
        self.edge_publish(ps)
        self.img_publish(img)

    def edge_publish(self, data):
        self.edge_pub.publish(data)

    def img_publish(self, data):
        self.img_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('hand_pos')
    node = hand_pos()
    rospy.spin()
