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

class ServoGear():

    def __init__(self,
                 camera,
                 debug_mode,
                 **kwargs):
        
        self.camera = camera
        self.recognition_object = "servo_gear"
        self.debug_mode = debug_mode
        self.sub_image = None
        self.gray_img = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.diff_before = None
        self.diff_after = None
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.pub_debug_image_raw = rospy.Publisher(self.camera + "/" + self.recognition_object + "/debug_image_raw", Image, queue_size=10)
        self.pub_equ_image = rospy.Publisher(self.camera + "/" + self.recognition_object + "/equalizehist_image", Image, queue_size=10)
        self.pub_threshold_image = rospy.Publisher(self.camera + "/" + self.recognition_object + "/threshold_image", Image, queue_size=10)
        # self.pub_ellipse_image = rospy.Publisher(self.camera + "/upper_arm_hole/ellipse_image", Image, queue_size=10)
        # self.pub_background_image = rospy.Publisher(self.camera + "/upper_arm_hole/background_image", Image, queue_size=10)
        # self.pub_diff_image = rospy.Publisher(self.camera + "/upper_arm_hole/diff_image", Image, queue_size=10)
        # self.pub_roi_image = rospy.Publisher(self.camera + "/upper_arm_hole/roi_image", Image, queue_size=10)
        # self.pub_masked_image = rospy.Publisher(self.camera + "/upper_arm_hole/masked_image", Image, queue_size=10)
        # self.pub_ellipse = rospy.Publisher(self.camera + "/upper_arm_hole/ellipse", RotatedRectStamped, queue_size=10)

        # self.service_name = "/" + camera + "/set_background"
        # self.service = rospy.Service(self.service_name, Empty, self.set_backgound_image)

        self.subscribe()
        
    def subscribe(self):
        sub_image = rospy.Subscriber(self.camera + "/color/image_rect_color", Image, self.cb)

    def set_backgound_image(self, req):
        rospy.loginfo("set background image")
        self.background_image = self.gray_img.copy()
        self.background_is_set = True
        return EmptyResponse()
        
    def cb(self, image):
        self.sub_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self.header = image.header

    def get_before_image(self):
        while self.diff_before is None:
            if (self.sub_image is not None):
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
        
                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                self.diff_before = equ_img_clip

                
    def get_after_image(self):
        while self.diff_after is None:
            if (self.sub_image is not None):
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
        
                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                self.diff_after = equ_img_clip

                
    def set_part_roi(self):
        diff = cv2.absdiff(self.diff_before, self.diff_after)
        cv2.imwrite("/home/amabe/rosbag/diff.png", diff)
        ret, diff = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)
        cv2.imwrite("/home/amabe/rosbag/diff_th.png", diff)

        
    def run(self):
        rate = rospy.Rate(10)

        rospy.loginfo("start to recognize servo gear")

        # rospy.loginfo("Waiting for service call...")
        # rospy.wait_for_service(self.service_name)
        # rospy.loginfo("Service is now available")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass
            
            if self.sub_image is not None:
                #pdb.set_trace()
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
                self.gray_img = gray_img.copy()

                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                ret, threshold_image = cv2.threshold(equ_img_clip, 120, 255, cv2.THRESH_BINARY)

                self.pub_debug_image_raw.publish(self.bridge.cv2_to_imgmsg(self.sub_image, "bgr8"))
                if self.debug_mode == "debug":
                    self.pub_equ_image.publish(self.bridge.cv2_to_imgmsg(equ_img_clip))
                    self.pub_threshold_image.publish(self.bridge.cv2_to_imgmsg(threshold_image))
                
            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("module_name", type=str, help="module_name")
    parser.add_argument("--debug", type=str, help="flag to launch in debug mode")
    
    rospy.init_node("servo_gear", anonymous=True)

    args = parser.parse_args()

    if args.debug == "True" or args.debug == "true":
        print("launch in debug mode")
        servo_gear = ServoGear(args.module_name, "debug")
    else:
        print("launch in normal mode")
        servo_gear = ServoGear(args.module_name, "normal")

    servo_gear.run()
    
