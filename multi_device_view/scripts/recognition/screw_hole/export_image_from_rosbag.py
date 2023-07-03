import rosbag
import cv2
import os
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

timestamps = []
images = []

image_topic = "/camera/color/image_rect_color"
output_dir = "./image_screw_d405"
bag_file = "screw_hole_d405.bag"

if not os.path.isdir(output_dir):
    os.mkdir(output_dir)

for topic, msg, t in  rosbag.Bag(bag_file).read_messages():
    if topic == image_topic:
        timestamps.append(t.to_sec())
        images.append( CvBridge().imgmsg_to_cv2(msg, "8UC3") )

# for i in range(len(images)):
#     cv2.imwrite(os.path.join(output_dir, "screw_%04i.png" % i), images[i])

for i in range(len(images)//50):
    cv2.imwrite(os.path.join(output_dir, "screw_%04i.png" % i), images[i*50])
