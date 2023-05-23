import rosbag
import cv2
import os
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

timestamps = []
images = []
count = 0
image_topic = "/timercam_2/timer_cam_image/image_rect_color"
output_dir = "."
bag_file = "/home/amabe/rosbag/0523_estimated_tf.bag"

# parser = argparse.ArgumentParser()
# parser.add_argument("--bagfile" )
# parser.add_argument("--output_dir", default=".")
# #parser.add_argument("--image_topic")
# #parser.add_argument("--save_name", default="frame")
# args = parser.parse_args()

for topic, msg, t in  rosbag.Bag(bag_file).read_messages():
    if topic == image_topic:
        timestamps.append(t.to_sec())
        images.append( CvBridge().imgmsg_to_cv2(msg, "8UC3") )

for i in range(len(images)):
    cv2.imwrite(os.path.join(output_dir, "timercam2_%06i.png" % i), images[i])
