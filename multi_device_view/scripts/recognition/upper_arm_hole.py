import numpy as np
import rospy
import skrobot
import sys
import tf
import tf2_ros
import time
import math
import cv2
import pdb
from cv_bridge import CvBridge
from numpy import pi

from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from dynamic_tf_publisher.srv import SetDynamicTF
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse

class UpperArmHole():

    def __init__(self,
                 camera,
                 **kwargs):
        
        self.camera = camera
        self.sub_image = None
        self.debug_image_raw = None
        self.bridge = CvBridge()
        self.diff_before = None
        self.diff_after = None
        self.capture_hole = False
        self.background_image = None
        self.background_is_set = False
        self.pub_debug_image_raw = rospy.Publisher(self.camera + "/upper_arm_hole/debug_image_raw", Image, queue_size=10)
        self.pub_equ_image = rospy.Publisher(self.camera + "/upper_arm_hole/equalizehist_image", Image, queue_size=10)
        self.pub_threshold_image = rospy.Publisher(self.camera + "/upper_arm_hole/threshold_image", Image, queue_size=10)
        self.pub_ellipse_image = rospy.Publisher(self.camera + "/upper_arm_hole/ellipse_image", Image, queue_size=10)
        self.pub_background_image = rospy.Publisher(self.camera + "/upper_arm_hole/background_image", Image, queue_size=10)
        self.trace_hole_image = rospy.Publisher(self.camera + "/upper_arm_hole/trace_hole_image", Image, queue_size=10)

        self.service_name = "/" + camera + "/set_background"
        self.service = rospy.Service(self.service_name, Empty, self.set_backgound_image)

        self.subscribe()

        
    def subscribe(self):
        sub_image = rospy.Subscriber(self.camera + "/color/image_rect_color", Image, self.cb)

    def set_backgound_image(self, req):
        rospy.loginfo("set background image")
        self.background_image = self.sub_image.copy()
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
        ret, diff = cv2.threshold(diff, 45, 255, cv2.THRESH_BINARY)
        cv2.imwrite("/home/amabe/rosbag/diff_th.png", diff)

    
        

        
    def run(self):
        rate = rospy.Rate(10)

        rospy.loginfo("Waiting for service call...")
        rospy.wait_for_service(self.service_name)
        rospy.loginfo("Service is now available")
        
        while not rospy.is_shutdown():
            try:
                rate.sleep()
                
            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

            self.capture_hole = True
            if((self.sub_image is not None) and (self.capture_hole == True)):
                #pdb.set_trace()
                gray_img = cv2.cvtColor(self.sub_image, cv2.COLOR_BGR2GRAY)
                gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

                equ_hist = cv2.equalizeHist(gray_img)
                equ_img = np.hstack((gray_img, equ_hist))
                equ_height, equ_width = equ_img.shape
                equ_img_clip = equ_img[:, equ_width//2:equ_width]

                ret, threshold_image = cv2.threshold(equ_img_clip, 25, 255, cv2.THRESH_BINARY)

                self.pub_debug_image_raw.publish(self.bridge.cv2_to_imgmsg(self.sub_image, "bgr8"))
                self.pub_equ_image.publish(self.bridge.cv2_to_imgmsg(equ_img_clip))
                self.pub_threshold_image.publish(self.bridge.cv2_to_imgmsg(threshold_image))

                if self.background_is_set:
                    ellipse_contours, _ = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                    ellipse_drawed_image = self.sub_image.copy()
                    #pdb.set_trace()
                    ellipse_list = []
                    for i, cnt in enumerate(ellipse_contours):
                        if len(cnt) >= 5: 
                            ellipse = cv2.fitEllipse(cnt)
                            if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                                ellipse_list.append(ellipse)
                                #nan is not eliminated
                                cx = int(ellipse[0][0])
                                cy = int(ellipse[0][1])
                                # pdb.set_trace()
                                try:
                                    ellipse_drawed_image  = cv2.ellipse(ellipse_drawed_image, ellipse, (255,0,0),2)
                                except Exception as e:
                                    pdb.set_trace()

                    
                    self.pub_ellipse_image.publish(self.bridge.cv2_to_imgmsg(ellipse_drawed_image, "bgr8"))
                
                    self.trace_hole_image.publish(self.bridge.cv2_to_imgmsg(ellipse_drawed_image, "bgr8"))
                    
                    if self.background_image is not None:
                        self.pub_background_image.publish(self.bridge.cv2_to_imgmsg(self.background_image, "bgr8"))
                
            else:
                rospy.logwarn("not recieve image")


if __name__ == "__main__":
    rospy.init_node("upper_arm_hole", anonymous=True)
    upper_arm_hole = UpperArmHole("module_0")

    upper_arm_hole.run()
    # upper_arm_hole_subscriber = rospy.Subsciriber("/module_0/color/image_rect_color", sensor/Image, upper_arm_hole.cb)
    
