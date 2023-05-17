import rospy
from subscribe_timercam_tf import Calculation_timercam_pos
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
args = parser.parse_args()

if args.test:
    frame_name = "base_link"
else:
    frame_name = "timer_cam1_optical_frame"

rospy.init_node("sucscribe_timercam_1_tf")
calculation_timercam_pos_1 = Calculation_timercam_pos(timercam_name = "timer_cam1", timercam_camera_frame = frame_name)
