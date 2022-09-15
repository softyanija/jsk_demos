import rospy
import cv2
import numpy as np
import time
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class memory_edge():
    def __init__(self):
        self.sub = rospy.Subscriber("/timer_cam2_rec/memory/hsv_color_filter/image", Image, self.callback)
        self.edge_pub = rospy.Publisher("/timer_cam2_rec/memory_edge", PoseArray, queue_size=3)
        self.img_pub = rospy.Publisher("/timer_cam2_rec/memory_edge/debug_image", Image, queue_size=3)
        
    def callback(self, data):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow('gray', img)
        ret,thresh = cv2.threshold(img_gray,127,255,0)
        imgEdge,contours,hierarchy = cv2.findContours(thresh, 1, 2)
        cnt = max(contours, key=lambda x: cv2.contourArea(x))
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print(box)
        buf = box.tolist()
        print(buf)
        img = cv2.drawContours(img, [box], 0, (0,255,0), 3)
        imgmsg = bridge.cv2_to_imgmsg(img, "bgr8")

        ps = PoseArray()
        ps.header.frame_id="timer_cam2"
        ps.header.stamp = rospy.Time.now()
        for i in range(4):
            pose = Pose()
            pose.position.x = box[i][0]
            pose.position.y = box[i][1]
            pose.position.z= 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            ps.poses.append(pose)
        
        self.edge_publish(ps)
        self.img_publish(imgmsg)

    def edge_publish(self, data):
        self.edge_pub.publish(data)

    def img_publish(self, data):
        self.img_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('pub_memory_edge')

    time.sleep(2.0)
    rate = rospy.Rate(1)
    node = memory_edge()

    while not rospy.is_shutdown():
        rate.sleep()
