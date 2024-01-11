import cv2
import numpy as np
import rospy
import time
import tf2_ros
import skrobot
import message_filters
import os

from skrobot.coordinates import Coordinates
from jsk_recognition_msgs.msg import RotatedRectStamped
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from scipy import linalg


class StereoView():
    # def __init__(self, camera1, camera2, t_1w, t_2w, guide_object, target_object, **kwargs):
    def __init__(self, target_object, guide_object, **kwargs):
        self.target_object = target_object
        self.guide_object = guide_object
        self.target_kp_1 = None
        self.target_kp_2 = None
        self.guide_kp_1 = None
        self.guide_kp_2 = None
        self.camera_1 = "module_0"
        self.camera_2 = "module_1"
        self.K_1 = None
        self.K_2 = None
        self.module_0_tf = None
        self.module_1_tf = None
        self.T_1w = None
        self.T_2w = None
        self.P_1w = None
        self.P_2w = None

        self.subscribe_kp()
        self.subscribe_camera_info()

        self.pub_target_point = rospy.Publisher(os.path.join("target_point", "3d_point"), PointStamped, queue_size=10)
        self.pub_guide_point = rospy.Publisher(os.path.join("guide_point", "3d_point"), PointStamped, queue_size=10)
        

    def subscribe_target_kp(self):
        sub_target_1 = message_filters.Subscriber(os.path.join(self.camera_1, self.target_object, target_point), RotatedRectStamped)
        sub_target_2 = message_filters.Subscriber(os.path.join(self.camera_2, self.target_object, target_point), RotatedRectStamped)
        
        subs = [sub_target_1, sub_target_2]
        sync = message_filters.ApproximateTimeSynchronizer(fs = subs, queue_size=5, slop=1)
        sync.registerCallback(self.target_keypoint_cb)
        

    def target_keypoint_cb(self, target_1, target_2):
        self.target_kp1 = np.array([[target_1.rect.x + target_1.rect.width / 2], [target_1.rect.y + 10]])
        self.target_kp2 = np.array([[target_2.rect.x + target_2.rect.width / 2], [target_2.rect.y + 10]])


    def subscribe_guide_kp(self):
        sub_guide_1 = message_filters.Subscriber(os.path.join(self.camera_1, self.guide_object, guide_point), RotatedRectStamped)
        sub_guide_2 = message_filters.Subscriber(os.path.join(self.camera_2, self.guide_object, guide_point), RotatedRectStamped)
        
        subs = [sub_guide_1, sub_guide_2]
        sync = message_filters.ApproximateTimeSynchronizer(fs = subs, queue_size=5, slop=1)
        sync.registerCallback(self.guide_keypoint_cb)
        

    def guide_keypoint_cb(self, guide_1, guide_2):
        self.guide_kp1 = np.array([[guide_1.rect.x + guide_1.rect.width / 2], [guide_1.rect.y]])
        self.guide_kp2 = np.array([[guide_2.rect.x + guide_2.rect.width / 2], [guide_2.rect.y]])


    def subscribe_camera_info(self):
        sub_camera_info_1 = message_filters.Subscriber(self.camera_1 + "/color/camera_info", CameraInfo)
        sub_camera_info_2 = message_filters.Subscriber(self.camera_2 + "/color/camera_info", CameraInfo)
        
        subs = [sub_camera_info_1, sub_camera_info_2]
        sync = message_filters.ApproximateTimeSynchronizer(fs = subs, queue_size=5, slop=1)
        sync.registerCallback(self.camera_info_cb)
        

    def camera_info_cb(self, camera_info_1, camera_info_2):
        self.K_1 = np.asarray(camera_info_1.K).reshape(3,3)
        self.K_2 = np.asarray(camera_info_2.K).reshape(3,3)

        
    def triangulation(self, P_1w, P_2w, kp1, kp2):    
        X = cv2.triangulatePoints(P_1w[:3], P_2w[:3], kp1, kp2)
        X = X / X[3]
        X1 = P_1w[:3] @ X
        X2 = P_2w[:3] @ X

        # return X[:3], X1, X2
        return X[:3]


    def tf2mat(msg):
        if msg is None:
            rospy.logwarn("can't recieve tf")

            tf = msg.transform
            tf_coordinate = skrobot.coordinates.Coordinates([tf.translation.x, tf.translation.y, tf.translation.z], [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])
            mat = tf_coordinate.T()[:3]

        else:
            mat = None
    
        return mat


    def run(self):
        rate = rospy.Rate(5)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        self.module_0_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_0_color_optical_frame", rospy.Time(0), rospy.Duration(3))
        self.module_1_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_1_color_optical_frame", rospy.Time(0), rospy.Duration(3))
        time.sleep(2)
        self.T_1w = tf2mat(module_0_tf)
        self.T_2w = tf2mat(module_1_tf)
        self.P_1w = np.dot(stereo_view.K_1, T_1w)
        self.P_2w = np.dot(stereo_view.K_2, T_2w)

        while not rospy.is_shutdown():
            try:
                rate.sleep()

            except rospy.ROSTimeMovedBackwardsException as e:
                rospy.logwarn("cought {}".format(e))
                pass

        target_point_msg = PointStamped()
        target_point_msg.header.stamp =rospy.Time.now() 
        target_point_msg.header.frame_id = "module_0_color_optical_frame"
        guide_point_msg = PointStamped()
        guide_point_msg.header.stamp =rospy.Time.now() 
        guide_point_msg.header.frame_id = "module_0_color_optical_frame"
            
        if self.target_kp1 is not None:
            X_target = stereo_view.triangulation(self.P_1w, self.P_2w, self.target_kp_1, self.target_kp_2)
            target_point_msg.point.x = X[0]
            target_point_msg.point.y = X[1]
            target_point_msg.point.z = X[2]

        if self.guide_kp1 is not None:
            guide_point_msg.point.x = X[0]
            guide_point_msg.point.y = X[1]
            guide_point_msg.point.z = X[2]

        pub_target_point(target_point_msg)
        pub_guide_point(guide_point_msg)        


if __name__ == "__main__":
    rospy.init_node("stereo_view")
    rate = rospy.Rate(5)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    stereo_view = StereoView("module_0/servo_gear/target_point", "module_1/servo_gear/target_point")

    module_0_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_0_color_optical_frame", rospy.Time(0), rospy.Duration(3))
    module_1_tf = tf_buffer.lookup_transform("module_0_color_optical_frame", "module_1_color_optical_frame", rospy.Time(0), rospy.Duration(3))
    time.sleep(2)
    T_1w = tf2mat(module_0_tf)
    T_2w = tf2mat(module_1_tf)
    P_1w = np.dot(stereo_view.K_1, T_1w)
    P_2w = np.dot(stereo_view.K_2, T_2w)

    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException as e:
            rospy.logwarn("cought {}".format(e))
            pass
        
        X, X1, X2 = stereo_view.triangulation(P_1w, P_2w, stereo_view.kp_1, stereo_view.kp_2)

        # X = stereo_view.DLT(P_1w, P_2w, stereo_view.kp1, stereo_view.kp2)
        point_msg = PointStamped()
        point_msg.header.stamp =rospy.Time.now() 
        point_msg.header.frame_id = "module_0_color_optical_frame"
        # point_msg.header.frame_id = "base_link"

        point_msg.point.x = X[0]
        point_msg.point.y = X[1]
        point_msg.point.z = X[2]

        # point_msg.point.x = X1[0]
        # point_msg.point.y = X1[1]
        # point_msg.point.z = X1[2]
        
        point_pub.publish(point_msg)

