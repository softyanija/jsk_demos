import numpy as np
import rospy
import sys
import tf
import tf2_ros
import time
import math
import cv2
import pdb
from cv_bridge import CvBridge
from numpy import pi
import argparse

from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_recognition_msgs.msg import RotatedRectStamped
from dynamic_tf_publisher.srv import SetDynamicTF
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from multi_device_view.msg import Ellipse
from dynamic_reconfigure.server import Server


class ServoGear():

    def __init__(self,
                 camera,
                 debug_mode,
                 **kwargs):
        
        self.camera = camera
        self.recognition_object = "servo_gear"
        self.debug_mode = False
        self.sub_color_image = None
        self.sub_depth_image = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.diff_before = None
        self.diff_after = None
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.pub_debug_image_raw = rospy.Publisher(self.camera + "/" + self.recognition_object + "/debug_image_raw", Image, queue_size=10)
        self.pub_debug_depth_image = rospy.Publisher(self.camera + "/" + self.recognition_object + "/debug_depth_image", Image, queue_size=10)
        self.pub_mask_depth = rospy.Publisher(self.camera + "/" + self.recognition_object + "/mask_depth", Image, queue_size=10)
        self.pub_mask_depth_test = rospy.Publisher(self.camera + "/" + self.recognition_object + "/mask_depth_test", Image, queue_size=10)
        self.pub_mask_applied_image = rospy.Publisher(self.camera + "/" + self.recognition_object + "/mask_applied_image", Image, queue_size=10)

        self.subscribe()


    def subscribe(self):
        sub_color_image = rospy.Subscriber(self.camera + "/color/image_rect_color", Image, self.color_cb)
        sub_depth_image = rospy.Subscriber(self.camera + "/depth/image_rect_raw", Image, self.depth_cb)
        sub_mask_image = rospy.Subscriber(self.camera + "/" + self.recognition_object + "/multiply_mask_image/output", Image, self.mask_cb)


    def color_cb(self, image):
        self.sub_color_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.color_header = image.header

    def depth_cb(self, image):
        self.sub_depth_image = self.bridge.imgmsg_to_cv2(image, "16UC1")
        self.depth_header = image.header

    def mask_cb(self, image):
        self.sub_mask_image = self.bridge.imgmsg_to_cv2(image, "16UC1")
        self.mask_header = image.header

    def run(self):
        rate = rospy.Rate(5)

        rospy.loginfo("start to recognize servo gear")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
            
            if (self.sub_color_image is not None) and (self.sub_depth_image is not None) and (self.sub_mask_image is not None):
                #pdb.set_trace()
                '''
                debug_image_raw_msg = self.bridge.cv2_to_imgmsg(self.sub_color_image, "bgr8")
                debug_image_raw_msg.header = self.header
                self.pub_debug_image_raw.publish(debug_image_raw_msg)                
                self.pub_debug_depth_image.publish(self.bridge.cv2_to_imgmsg(self.sub_depth_image, "16UC1"))

                mask_depth_msg = self.bridge.cv2_to_imgmsg(mask_depth, "8UC1")
                mask_depth_msg.header = self.mask_header
                
                self.pub_mask_depth.publish(mask_depth_msg)
                '''
                #self.pub_mask_depth_test.publish(self.bridge.cv2_to_imgmsg(mask_depth_test, "16UC1"))
                mask = self.sub_mask_image.astype(np.uint8)
                mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                #pdb.set_trace()
                mask_applied_image = cv2.bitwise_and(self.sub_color_image, mask_color)
                self.pub_mask_applied_image.publish(self.bridge.cv2_to_imgmsg(mask_applied_image, "bgr8"))
                
            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("module_name", type=str, help="module_name")  
    rospy.init_node("servo_gear", anonymous=True)

    args = parser.parse_args()
    
    servo_gear = ServoGear(args.module_name, "normal")

    servo_gear.run()
    
