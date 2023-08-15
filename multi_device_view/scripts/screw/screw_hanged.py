import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time
import math

from skrobot.coordinates import CascadedCoords
from skrobot.coordinates import Coordinates
from skrobot.coordinates.quaternion import Quaternion
from skrobot.interfaces.ros import PR2ROSRobotInterface
from skrobot.interfaces.ros.tf_utils import tf_pose_to_coords
from skrobot.interfaces.ros.tf_utils import geometry_pose_to_coords
from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point, TransformStamped
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import Marker
from dynamic_tf_publisher.srv import SetDynamicTF

class ScrewHanged():
    def __init__(self):
        self.msg = None
        self.start = None
        self.end = None
        self.tip_frame = None
        self.tool_frame_to_tip = 0.05
        self.tool_frame = None
        self.tip_length = 0.00
        self.range_limit = 0.008
        # self.driver_tip_frame = None 
        self.rate = 5
        self.tf_hz = 10
        
    def calc_tip(self):
        
        # if self.msg.boxes[0].pose.position.x == 0.0 and self.msg.boxes[0].pose.position.y == 0.0 and self.msg.boxes[0].pose.position.z == 0.0:
        # return
        print("hoge")
        
        pose_vector = Quaternion(self.msg.boxes[0].pose.orientation.w,
                                 self.msg.boxes[0].pose.orientation.x,
                                 self.msg.boxes[0].pose.orientation.y,
                                 self.msg.boxes[0].pose.orientation.z)

        tip_frame_rotation = skrobot.coordinates.Coordinates([0, 0, 0],
                                                        [self.msg.boxes[0].pose.orientation.w,
                                                         self.msg.boxes[0].pose.orientation.x,
                                                         self.msg.boxes[0].pose.orientation.y,
                                                         self.msg.boxes[0].pose.orientation.z])
        trans_from_screw = np.array([self.msg.boxes[0].dimensions.x / 2, 0, 0])
        tip_trans = np.dot(tip_frame_rotation.rotation, trans_from_screw)
        
        axis_convert = skrobot.coordinates.Coordinates([0, 0, 0], [1, 0, 0, 0])
        tip_frame_buf = skrobot.coordinates.Coordinates([self.msg.boxes[0].pose.position.x + tip_trans[0],
                                                         self.msg.boxes[0].pose.position.y + tip_trans[1],
                                                         self.msg.boxes[0].pose.position.z + tip_trans[2]],
                                                        [self.msg.boxes[0].pose.orientation.w,
                                                         self.msg.boxes[0].pose.orientation.x,
                                                         self.msg.boxes[0].pose.orientation.y,
                                                         self.msg.boxes[0].pose.orientation.z])
        print(tip_trans)
        
        self.tip_frame = tip_frame_buf.copy_worldcoords().transform(axis_convert)

    def pub_tip_frame_tf(self):
        self.calc_tip()
        tip_tf = TransformStamped()
        tip_tf.header.frame_id = "camera_color_optical_frame"
        tip_tf.child_frame_id = "screw_induction_frame"
        tip_tf.transform.translation.x = self.tip_frame.translation[0]
        tip_tf.transform.translation.y = self.tip_frame.translation[1]
        tip_tf.transform.translation.z = self.tip_frame.translation[2]
        tip_tf.transform.rotation.x = self.tip_frame.quaternion[1]
        tip_tf.transform.rotation.y = self.tip_frame.quaternion[2]
        tip_tf.transform.rotation.z = self.tip_frame.quaternion[3]
        tip_tf.transform.rotation.w = self.tip_frame.quaternion[0]
        
        rospy.wait_for_service("/set_dynamic_tf")
        try:
            client = rospy.ServiceProxy("/set_dynamic_tf", SetDynamicTF)
            res = client(self.tf_hz, tip_tf)
            return
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def cb(self, msg):
        self.msg = msg
        

if __name__ == "__main__":
    rospy.init_node("screw_hanged")
    screw_hanged = ScrewHanged()
    screw_hanged_subscriber = rospy.Subscriber("/screw/euclidean_clustering_decomposer/boxes", BoundingBoxArray, screw_hanged.cb)
    rospy.sleep(1)
    #screw_hanged.get_driver_tip_frame()
    screw_hanged.select_line()
    screw_hanged.calc_tip()
    screw_hanged.pub_tip_frame_tf()


