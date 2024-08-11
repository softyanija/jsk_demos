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

class MemoryEdgeCam2():

    def __init__(self):
        self.subs = []
        self.header = None
        self.sub_image = None
        self.result_image = None
        self.bridge = CvBridge()

        self.pub_image = rospy.Publisher("/timer_cam2_rec/memory_edge/debug_image", Image, queue_size=3)
        self.pub_guid_point_b = rospy.Publisher("/timer_cam2_rec/memory_edge/guide_point_b", PoseArray, queue_size=3)
        self.pub_memory_edge = rospy.Publisher("/timer_cam2_rec/memory_edge/memory_edge", PoseArray, queue_size=3)
        self.subscribe()


    def subscribe(self):
        self.sub = rospy.Subscriber("/timer_cam2_rec/memory/hsv_color_filter/image", Image, self.callback)


    def callback(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "rgb8")
        self.header = image.header


    def run(self):
        rate = rospy.Rate(3)
        rospy.loginfo("start indicate_memory_edge_cam1")

        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            if (self.sub_image is not None):
                self.result_image = self.sub_image.copy()
                self.memory_edge = PoseArray()
                self.memory_edge.header = self.header
                self.memory_guide_point_b = PoseArray()
                self.memory_guide_point_b.header = self.header
                img_gray = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(img_gray,127,255,0)
                contours,hierarchy = cv2.findContours(thresh, 1, 2)
        
                if (not contours == []):
                    cnt = max(contours, key=lambda x: cv2.contourArea(x))
                    rect = cv2.minAreaRect(cnt)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    buf = box.tolist()
                    # img = cv2.drawContours(img, [box], 0, (0,255,0), 3)
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
                        self.result_image = cv2.circle(self.result_image, (pose.position.x, pose.position.y), 2, (0,255,0), 3,4,0)
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
                    self.result_image = cv2.circle(self.result_image, (self.memory_edge.poses[use_index].position.x, self.memory_edge.poses[use_index].position.y), 4, (255,0,0), 3,4,0)
                self.result_image = self.bridge.cv2_to_imgmsg(self.result_image, "rgb8")

                self.pub_image.publish(self.result_image)

                self.pub_memory_edge.publish(self.memory_edge)
                self.pub_guid_point_b.publish(self.memory_guide_point_b)
                rospy.loginfo("memory edge x: " + str(self.memory_edge.poses[use_index].position.x) + " y: " + str(self.memory_edge.poses[use_index].position.y))
            else:
                rospy.logwarn("didn't recieve image")


if __name__ == '__main__':
    rospy.init_node('memory_edge_cam2')
    memory_edge_cam2 = MemoryEdgeCam2()
    memory_edge_cam2.run()
