import numpy as np
import rospy
import skrobot
import tf
import tf2_ros
import time


from geometry_msgs.msg import PoseStamped, PoseArray, WrenchStamped, Point
from visualization_msgs.msg import Marker

class Driver():
    def __init__(self):
        self.msg = None
        self.start_point = None
        self.end_point = None
        self.tip_frame = None
        self.rate = 5
        #self.subscribe

    # def subscribe(self):
    #     rospy.Subscriber("/driver/line_segment_detector/debug/line_marker", Marker, self.cb)
    #     rate = rospy.Rate(self.rate)
    #     rate.sleep()

    def select_line(self):
        if len(self.msg.points) == 0:
            return
        center_list = []
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        tool_frame = tf_buffer.lookup_transform("camera_color_optical_frame", "r_gripper_tool_frame", rospy.Time(), rospy.Duration(3))
        tool_frame_center = Point()
        tool_frame_center.x = tool_frame.transform.translation.x
        tool_frame_center.y = tool_frame.transform.translation.y
        tool_frame_center.z = tool_frame.transform.translation.z

        for i in range(int(len(self.msg.points)/2)):
            center_buf = Point()
            center_buf.x = (self.msg.points[2*i].x + self.msg.points[2*i].x)/2
            center_buf.y = (self.msg.points[2*i].y + self.msg.points[2*i].y)/2
            center_buf.z = (self.msg.points[2*i].z + self.msg.points[2*i].z)/2
            center_list.append(center_buf)

        closest_i  = 0
        min_range = float("inf")
        for i in range(len(center_list)):
            range_buf = (tool_frame_center.x - center_list[i].x)**2 + (tool_frame_center.y - center_list[i].y)**2 + (tool_frame_center.z - center_list[i].z)**2
            if min_range > range_buf:
                closest_i = i
                min_range = range

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
        
        
        

    def cb(self, msg):
        self.msg = msg
        

if __name__ == "__main__":
    rospy.init_node("driver")
    driver = Driver()
    driver_subscriber = rospy.Subscriber("/driver/line_segment_detector/debug/line_marker", Marker, driver.cb)
