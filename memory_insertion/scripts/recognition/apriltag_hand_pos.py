#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from cv_bridge import CvBridge


class ApriltagHandPos():

    def __init__(self):
        self.bridge = CvBridge()
        self.result_image = None
        self.hand_pos = None
        self.subscribe_image()
        self.subscribe_tags()

        # self.pub_hand_pos = rospy.Publisher("/timer_cam2_rec/hand_pos/position", PoseArray, queue_size=3)
        self.pub_image = rospy.Publisher("/timer_cam2_rec/hand_pos/debug_image", Image, queue_size=3)


    def subscribe_image(self):
        sub_image = rospy.Subscriber("/timer_cam2/timer_cam_image/image_rect_color", Image, self.image_callback)

    
    def image_callback(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")


    def subscribe_tags(self):
        self.sub_tags = rospy.Subscriber("/timer_cam2_rec/tag_detections", AprilTagDetectionArray, self.tags_callback)

        
    def tags_callback(self, tags):
        self.sub_tags = tags.detections


    def run(self):
        rate = rospy.Rate(2)
        self.hand_pos = PoseArray()
        self.hand_pos.header = image.header
        self.result_image = self.sub_image.copy()

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                
                if not self.tags == []:
                    tag_found = 0
                    for i, tag in enumerate(self.tags):
                        for id in tag.id:
                            if id == 6: # l_gripper_front_apriltag found
                                
                                tag.pose

        self.pub_image.publish(self.result_image)
        

if __name__ == '__main__':
    rospy.init_node('apriltag_hand_pos')
    apriltag_hand_pos = ApriltagHandPos()
    apriltag_hand_pos.run()
    

