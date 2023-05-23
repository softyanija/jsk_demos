import numpy as np
import cv2
import rospy
from geometry_msgs.msg import *
import tf
import tf2_ros
from tf2_msgs.msg import TFMessage
import quaternion


cameraMatrix1 = np.array([[471.99697,         0, 299.54147],
                          [0        , 471.4478 , 229.05121],
                          [0        ,         0,   1.     ]])
distCoeffs1 = np.array([[-0.331875, 0.089660, -0.002166, -0.000423, 0.000000]])
cameraMatrix2 = np.array([[489.44513,         0, 334.76048],
                          [0        , 488.25644, 232.22405],
                          [0        ,         0,   1.     ]])
distCoeffs2 = np.array([[-0.385310, 0.143555, 0.000299, 0.001975, 0.000000]])

rospy.init_node("stereo_test", anonymous=True)
rectify_scale = 0
# T = np.array([-1, 0, 0.5]).reshape(3, 1)

tf_buffer = tf2_ros.Buffer()
tf_listerner = tf2_ros.TransformListener(tf_buffer)

while(1):
    try:
        trans = tf_buffer.lookup_transform("timercam_2_optical_frame_estimated", "timercam_1_optical_frame_estimated", rospy.Time(), rospy.Duration(0.5))
        break
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Failed to get trans")        

q = np.quaternion(trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z)
R = quaternion.as_rotation_matrix(q)
T = np.array([[trans.transform.translation.x],
              [trans.transform.translation.y],
              [trans.transform.translation.z]])
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify( \
        cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, (640, 480), R, T, alpha=rectify_scale)
# prepare to remap the webcams
l_maps = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (640, 360), cv2.CV_16SC2)
r_maps = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (640, 360), cv2.CV_16SC2)

#i = 1
lframe = cv2.imread("calib_imgs/stereo_test/timercam2_000001.png", 1)
rframe = cv2.imread("calib_imgs/stereo_test/timercam1_000001.png", 1)
# use the rectified data to do remap on webcams
lframe_remap = cv2.remap(lframe, l_maps[0], l_maps[1], cv2.INTER_LINEAR)
rframe_remap = cv2.remap(rframe, r_maps[0], r_maps[1], cv2.INTER_LINEAR)
viz = np.concatenate([lframe_remap, rframe_remap], axis=1)
cv2.imshow('viz', viz)
cv2.waitKey()
cv2.destroyAllWindows()
