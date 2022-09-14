"""
import cv2
import numpy as np

im = cv2.imread('mask_image.jpg',0)
ret,thresh = cv2.threshold(im,127,255,0)
imgEdge,contours,hierarchy = cv2.findContours(thresh, 1, 2)
cnt = max(contours, key=lambda x: cv2.contourArea(x))
print(cnt)
rect = cv2.minAreaRect(cnt)
box = cv2.boxPoints(rect)
box = np.int0(box)
print(box)
output = cv2.drawContours(im,[box],0,(0,255,255),5)

cv2.imwrite('./memory_edge_o.jpg',output)
"""

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

def main():

    rospy.init_node("memory_edge_publisher")

    pub = rospy.Publisher("/memory_edge", PoseArray, queue_size=3)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        im = cv2.imread('mask_image.jpg',0)
        ret,thresh = cv2.threshold(im,127,255,0)
        imgEdge,contours,hierarchy = cv2.findContours(thresh, 1, 2)
        cnt = max(contours, key=lambda x: cv2.contourArea(x))
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        print(box)
        buf = box.tolist()
        print(buf)

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
        
        pub.publish(ps)
        rospy.loginfo
        rate.sleep()

if __name__ == "__main__":
    main()
