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

class Driver():
    def __init__(self):
        self.msg = None
        self.start_point = None
        self.end_point = None
        self.tip_frame = None
        self.tip_length = 0.001
        self.tool_frame_to_tip = 0.046
        self.tool_frame = None 
        self.rate = 5
        self.tf_hz = 10

    def get_tool_frame(self):
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        self.tool_frame = tf_buffer.lookup_transform("camera_color_optical_frame", "r_gripper_tool_frame", rospy.Time(), rospy.Duration(3))

    def select_line(self):
        if len(self.msg.points) == 0:
            return
        center_list = []
        self.get_tool_frame()
        
        tool_frame_center = Point()
        tool_frame_center.x = self.tool_frame.transform.translation.x
        tool_frame_center.y = self.tool_frame.transform.translation.y
        tool_frame_center.z = self.tool_frame.transform.translation.z

        for i in range(int(len(self.msg.points)/2)):
            center_buf = Point()
            center_buf.x = (self.msg.points[2*i].x + self.msg.points[2*i].x)/2
            center_buf.y = (self.msg.points[2*i].y + self.msg.points[2*i].y)/2
            center_buf.z = (self.msg.points[2*i].z + self.msg.points[2*i].z)/2
            center_list.append(center_buf)

        closest_i  = 0
        range_diff_min = float("inf")
        for i in range(len(center_list)):
            range_diff = abs(math.sqrt((tool_frame_center.x - center_list[i].x)**2 + (tool_frame_center.y - center_list[i].y)**2 + (tool_frame_center.z - center_list[i].z)**2) - self.tool_frame_to_tip)
            if range_diff_min > range_diff:
                closest_i = i
                range_diff_min = range_diff

        edge_1 = self.msg.points[2*closest_i]
        edge_2 = self.msg.points[2*closest_i + 1]
        range_1 = (tool_frame_center.x - edge_1.x)**2 + (tool_frame_center.y - edge_1.y)**2 + (tool_frame_center.z - edge_1.z)**2
        range_2 = (tool_frame_center.x - edge_2.x)**2 + (tool_frame_center.y - edge_2.y)**2 + (tool_frame_center.z - edge_2.z)**2

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
        
        self.get_tool_frame()
        axis_convert = skrobot.coordinates.Coordinates([0, 0, 0], [-0.707106781186547, 0, 0.707106781186547, 0])
        tip_frame_buf = skrobot.coordinates.Coordinates(tip_translation,
                                                         [self.tool_frame.transform.rotation.w,
                                                          self.tool_frame.transform.rotation.x,
                                                          self.tool_frame.transform.rotation.y,
                                                          self.tool_frame.transform.rotation.z])
        self.tip_frame = tip_frame_buf.copy_worldcoords().transform(axis_convert)

    def pub_tip_frame_tf(self):
        self.select_line()
        self.calc_tip()
        tip_tf = TransformStamped()
        tip_tf.header.frame_id = "camera_color_optical_frame"
        tip_tf.child_frame_id = "driver_tip_frame"
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
    rospy.init_node("driver")
    driver = Driver()
    driver_subscriber = rospy.Subscriber("/driver/line_segment_detector/debug/line_marker", Marker, driver.cb)
    rospy.sleep(1)
    driver.get_tool_frame()
    driver.select_line()
    driver.calc_tip()
    driver.pub_tip_frame_tf()
