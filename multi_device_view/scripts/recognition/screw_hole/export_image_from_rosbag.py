import rosbag
import cv2
import os
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

timestamps = []
images = []

image_topic = "/timercam_1/timer_cam_image/image_rect_color"
output_dir = "./images_screw_clip"
bag_file = "/home/amabe/rosbag/0621_screw.bag"

if not os.path.isdir(output_dir):
    os.mkdir(output_dir)

for topic, msg, t in  rosbag.Bag(bag_file).read_messages():
    if topic == image_topic:
        timestamps.append(t.to_sec())
        images.append( CvBridge().imgmsg_to_cv2(msg, "8UC3") )

for i in range(len(images)):
    cv2.imwrite(os.path.join(output_dir, "screw_%04i.png" % i), images[i])
