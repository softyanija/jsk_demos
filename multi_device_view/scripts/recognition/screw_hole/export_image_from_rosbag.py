import rosbag
import cv2
import os
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

timestamps = []
images = []
count = 0
image_topic = "/timercam_1/timer_cam_image/image_rect_color"
output_dir = "./images"
bag_file = "/home/amabe/rosbag/0620_recog_test.bag"

for topic, msg, t in  rosbag.Bag(bag_file).read_messages():
    if topic == image_topic:
        timestamps.append(t.to_sec())
        images.append( CvBridge().imgmsg_to_cv2(msg, "8UC3") )

for i in range(len(images)):
    cv2.imwrite(os.path.join(output_dir, "screw_hole_%06i.png" % i), images[i])
