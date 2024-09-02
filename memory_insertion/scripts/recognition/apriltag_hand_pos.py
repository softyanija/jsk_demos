#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge
import skrobot
from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion


class ApriltagHandPos():

    def __init__(self):
        self.bridge = CvBridge()
        self.result_image = None
        self.header = None
        self.hand_pos = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.tag_size = 0.015
        self.sub_tags = None
        self.subscribe_image()
        self.subscribe_camera_info()
        self.subscribe_tags()

        # self.pub_hand_pos = rospy.Publisher("/timer_cam2_rec/hand_pos/position", PoseArray, queue_size=3)
        self.pub_image = rospy.Publisher("/timer_cam2_rec/hand_pos/debug_image", Image, queue_size=3)


    def subscribe_image(self):
        sub_image = rospy.Subscriber("/timer_cam2/timer_cam_image/image_rect_color", Image, self.image_callback)


    def image_callback(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header

    def subscribe_camera_info(self):
        sub_camera_info = rospy.Subscriber("/timer_cam2/timer_cam_image/camera_info", CameraInfo, self.camera_info_callback)


    def camera_info_callback(self, info):
        self.camera_matrix = np.array(info.K).reshape(3,3)
        self.dist_coeffs = np.array(info.D)


    def subscribe_tags(self):
        sub_tags = rospy.Subscriber("/timer_cam2_rec/timer_cam2/tag_detections", AprilTagDetectionArray, self.tags_callback)

        
    def tags_callback(self, tags):
        self.sub_tags = tags.detections


    def run(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            self.hand_pos = None
            if self.camera_matrix is None:
                rospy.logwarn("Camera_catrix is not yet availabe")
                continue

            if self.dist_coeffs is None:
                rospy.logwarn("Dist_coeffs is not yet availabe")
                continue

            if self.sub_image is None :
                rospy.logwarn("Camera image is not yet availabe")
                continue

            if self.sub_tags is None:
                rospy.logwarn("Tag is not yet availabe")
                continue

            self.hand_pos = PoseArray()
            self.hand_pos.header = self.header
            self.result_image = self.sub_image.copy()

            self.result_image = self.sub_image.copy()
                
            tag_found = 0
            for tag in self.sub_tags:
                for id in tag.id:
                    if id == 6: # l_gripper_front_apriltag found
                        tag_q = np.array([
                            tag.pose.pose.pose.orientation.w,
                            tag.pose.pose.pose.orientation.x,
                            tag.pose.pose.pose.orientation.y,
                            tag.pose.pose.pose.orientation.z
                        ])
                        self.rvec, _ = cv2.Rodrigues(skrobot.coordinates.Coordinates([0,0,0],tag_q).rotation)                        
                        self.tvec = np.array([
                            tag.pose.pose.pose.position.x,
                            tag.pose.pose.pose.position.y,
                            tag.pose.pose.pose.position.z
                        ])

                        corners_3d = np.array([
                            [-self.tag_size/2, -self.tag_size/2, 0],
                            [ self.tag_size/2, -self.tag_size/2, 0],
                            [ self.tag_size/2,  self.tag_size/2, 0],
                            [-self.tag_size/2,  self.tag_size/2, 0]
                        ])

                        corners_2d, _ = cv2.projectPoints(corners_3d, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)

                        x_sum = 0
                        y_sum = 0
                        for corner in corners_2d:
                            x, y = corner.ravel()
                            x_sum += x
                            y_sum += y
                            
                            self.result_image = cv2.circle(self.result_image, (int(x),int(y)), 3,(0,0,255),1,4,0)

                        self.hand_pos = (int(x_sum/4), int(y_sum/4))
                        self.result_image = cv2.circle(self.result_image, self.hand_pos, 3,(255,0,0),4,4,0)
                        rospy.loginfo(f"AprilTag center in image: x={int(self.hand_pos[0])}, y={self.hand_pos[1]}")

            self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")
            self.pub_image.publish(self.result_image)
        

if __name__ == '__main__':
    rospy.init_node('apriltag_hand_pos')
    apriltag_hand_pos = ApriltagHandPos()
    apriltag_hand_pos.run()
    

