import cv2
import sys
import pdb
import math
import numpy as np
import pyrealsense2 as rs

config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline = rs.pipeline()
profile = pipeline.start(config)

reference_img = cv2.imread("../template_image/servo_gear_template.png")
reference_h, reference_w, _ = reference_img.shape

try:
    while True:
        
        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        #pdb.set_trace()

        result = cv2.matchTemplate(color_image, reference_img,cv2.TM_CCORR_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
        cv2.minMaxLoc(result)
        
        top_left = max_loc
        bottom_right = (top_left[0] + reference_w, top_left[1] + reference_h)
        guided_point = (top_left[0] + reference_w // 2, top_left[1] )
        cv2.rectangle(color_image, top_left, bottom_right, (255, 255, 0), 2)
        cv2.circle(color_image, guided_point, 2, (0, 0, 255), -1)

        # cv2.imshow("equalize", equ_img_clip)

        cv2.imshow("color", color_image)
        cv2.imshow("matching_result", color_image)
        # cv2.imshow("ellipse", ellipse_drawed_image)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
    
finally:
    pdb.set_trace()
    pipeline.stop()
