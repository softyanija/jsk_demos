import cv2
import numpy as np
import rospy
import time
import tf2_ros
import skrobot
import message_filters

from skrobot.coordinates import Coordinates
from jsk_recognition_msgs.msg import RotatedRectStamped
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from scipy import linalg

class StereoView():
    # def __init__(self, camera1, camera2, t_1w, t_2w, guide_object, target_object, **kwargs):
    def __init__(self, target_1, target_2, **kwargs):
        self.target_1 = target_1
        self.target_2 = target_2
        self.kp_1 = None
        self.kp_2 = None
        self.camera_1 = "module_0"
        self.camera_2 = "module_1"
        self.K_1 = None
        self.K_2 = None

        self.subscribe()


    def keypoint_cb(self, target_1, target_2, camera_info_1, camera_info_2):
        self.kp1 = np.array([[target_1.rect.x + target_1.rect.width / 2], [target_1.rect.y + 10]])
        self.kp2 = np.array([[target_2.rect.x + target_2.rect.width / 2], [target_2.rect.y + 10]])
        self.K_1 = np.asarray(camera_info_1.K).reshape(3,3)
        self.K_2 = np.asarray(camera_info_2.K).reshape(3,3)


    def subscribe(self):
        sub_target_1 = message_filters.Subscriber(self.target_1, RotatedRectStamped)
        sub_target_2 = message_filters.Subscriber(self.target_2, RotatedRectStamped)
        sub_camera_info_1 = message_filters.Subscriber(self.camera_1 + "/color/camera_info", CameraInfo)
        sub_camera_info_2 = message_filters.Subscriber(self.camera_2 + "/color/camera_info", CameraInfo)
        
        subs = [sub_target_1, sub_target_2, sub_camera_info_1, sub_camera_info_2]
        sync = message_filters.ApproximateTimeSynchronizer(fs = subs, queue_size=5, slop=1)
        sync.registerCallback(self.keypoint_cb)

        
    def triangulation(self, P_1w, P_2w, kp1, kp2):
        """Triangulation to get 3D points
        Args:
        T_1w (4x4): pose of view 1 w.r.t  i.e. T_1w (from w to 1)
        T_2w (4x4): pose of view 2 w.r.t world, i.e. T_2w (from w to 2)
        　　[R(3), t(1)]
        kp1 (2,1): keypoint in view 1 (not normalized?)
        kp2 (2,1): keypoints in view 2 (not normalized?
        shape of T will be change by treatment of tf 
        
        Returns:
        X (3,1): 3D coordinates of the keypoints w.r.t world coordinate
        X1 (3,1): 3D coordinates of the keypoints w.r.t view1 coordinate
        X2 (3,1): 3D coordinates of the keypoints w.r.t view2 coordinate
        """
        
        X = cv2.triangulatePoints(P_1w[:3], P_2w[:3], kp1, kp2)
        X = X / X[3]
        X1 = P_1w[:3] @ X
        X2 = P_2w[:3] @ X
        return X[:3], X1, X2

    def DLT(self, P1, P2, point1, point2):
        A = [point1[1]*P1[2,:] - P1[1,:],
             P1[0,:] - point1[0]*P1[2,:],
             point2[1]*P2[2,:] - P2[1,:],
             P2[0,:] - point2[0]*P2[2,:],]
        A = np.array(A).reshape((4,4))
        B = np.dot(A.transpose(), A)
        U, s, Vh = linalg.svd(B, full_matrices=False)
        return Vh[3, 0:3] / Vh[3, 3]


def tf2mat(msg):
    if msg is None:
        rospy.logwarn("can't recieve tf")

    tf = msg.transform
    tf_coordinate = skrobot.coordinates.Coordinates([tf.translation.x, tf.translation.y, tf.translation.z], [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z])
    mat = tf_coordinate.T()[:3]
    
    return mat




if __name__ == "__main__":
    rospy.init_node("stereo_view")
    rate = rospy.Rate(5)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    point_pub = rospy.Publisher("stereo_debug_point", PointStamped, queue_size=5)

    stereo_view = StereoView("module_0/servo_gear/target_point", "module_1/servo_gear/target_point")

    # module_0_tf = tf_buffer.lookup_transform("base_link", "module_0_color_optical_frame", rospy.Time(0), rospy.Duration(3))
    # module_1_tf = tf_buffer.lookup_transform("base_link", "module_1_color_optical_frame", rospy.Time(0), rospy.Duration(3))
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
        
        X, X1, X2 = stereo_view.triangulation(P_1w, P_2w, stereo_view.kp1, stereo_view.kp2)

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
        
    
    
