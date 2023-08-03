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
        self.driver_tip_frame = None 
        self.rate = 5
        self.tf_hz = 10
    
    def get_driver_tip_frame(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        self.driver_tip_frame = tf_buffer.lookup_transform("camera_color_optical_frame", "driver_tip_frame", rospy.Time(), rospy.Duration(5))
        self.tool_frame = tf_buffer.lookup_transform("camera_color_optical_frame", "r_gripper_tool_frame", rospy.Time(), rospy.Duration(5))

    def select_line(self):
        count = 0
        while 1:
            self.get_driver_tip_frame()
            print("count :" + str(count))
            count += 1
            if len(self.msg.points) == 0:
                continue
            center_list = []
            self.get_driver_tip_frame()
        
            driver_tip_frame_center = Point()
            driver_tip_frame_center.x = self.driver_tip_frame.transform.translation.x
            driver_tip_frame_center.y = self.driver_tip_frame.transform.translation.y
            driver_tip_frame_center.z = self.driver_tip_frame.transform.translation.z

            for i in range(int(len(self.msg.points)/2)):
                center_buf = Point()
                center_buf.x = (self.msg.points[2*i].x + self.msg.points[2*i].x)/2
                center_buf.y = (self.msg.points[2*i].y + self.msg.points[2*i].y)/2
                center_buf.z = (self.msg.points[2*i].z + self.msg.points[2*i].z)/2
                center_list.append(center_buf)

            closest_i  = 0
            min_range = float("inf")
            for i in range(len(center_list)):
                range_buf = (driver_tip_frame_center.x - center_list[i].x)**2 + (driver_tip_frame_center.y - center_list[i].y)**2 + (driver_tip_frame_center.z - center_list[i].z)**2
                print("range of index:" + str(i) + " is " + str(range_buf))
                if min_range > range_buf:
                    closest_i = i
                    min_range = range_buf
            print("min_range is " +  str(min_range))
            if min_range < 0.002:
                break
            if count > 100:
                break
            
        edge_1 = self.msg.points[2*closest_i]
        edge_2 = self.msg.points[2*closest_i + 1]
        # range_1 = (driver_tip_frame_center.x - edge_1.x)**2 + (driver_tip_frame_center.y - edge_1.y)**2 + (driver_tip_frame_center.z - edge_1.z)**2
        # range_2 = (driver_tip_frame_center.x - edge_2.x)**2 + (driver_tip_frame_center.y - edge_2.y)**2 + (driver_tip_frame_center.z - edge_2.z)**2
        range_1 = edge_1.y
        range_2 = edge_2.y

        if range_1 < range_2:
            self.start = self.msg.points[2*closest_i]
            self.end = self.msg.points[2*closest_i + 1]
        else:
            self.start = self.msg.points[2*closest_i + 1]
            self.end = self.msg.points[2*closest_i]
        
    def calc_tip(self):
        start = skrobot.coordinates.Coordinates([self.start.x, self.start.y, self.start.z],
                                                [1, 0, 0, 0])
        end = skrobot.coordinates.Coordinates([self.end.x, self.end.y, self.end.z],
                                              [1, 0, 0, 0])
        direction = end.translation - start.translation
        normalized_direction = direction / np.linalg.norm(direction)
        tip_translation = end.translation + self.tip_length * normalized_direction
        
        self.get_driver_tip_frame()
        axis_convert = skrobot.coordinates.Coordinates([0, 0, 0], [1, 0, 0, 0])
        tip_frame_buf = skrobot.coordinates.Coordinates(tip_translation,
                                                         [self.driver_tip_frame.transform.rotation.w,
                                                          self.driver_tip_frame.transform.rotation.x,
                                                          self.driver_tip_frame.transform.rotation.y,
                                                          self.driver_tip_frame.transform.rotation.z])
        self.tip_frame = tip_frame_buf.copy_worldcoords().transform(axis_convert)

    def pub_tip_frame_tf(self):
        self.select_line()
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
    screw_hanged_subscriber = rospy.Subscriber("/screw/line_segment_detector/debug/line_marker", Marker, screw_hanged.cb)
    rospy.sleep(1)
    #screw_hanged.get_driver_tip_frame()
    screw_hanged.select_line()
    screw_hanged.calc_tip()
    screw_hanged.pub_tip_frame_tf()
