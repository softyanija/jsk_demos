import rospy
from subscribe_timercam_tf import Calculation_timercam_pos

rospy.init_node("sucscribe_timercam_1_tf")
Calculation_timercam_pos_1 = Calculation_timercam_pos(timercam_name = "timer_cam1")
