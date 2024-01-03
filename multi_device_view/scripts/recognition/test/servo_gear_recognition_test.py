import cv2
import sys
import pdb
import math
import numpy as np
import pyrealsense2 as rs

config = rs.config()

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 5)

pipeline = rs.pipeline()
profile = pipeline.start(config)

try:
    while True:
        
        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        #pdb.set_trace()

        gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.medianBlur(gray_img, 5)
    
        equ_hist = cv2.equalizeHist(gray_img)
        equ_img = np.hstack((gray_img, equ_hist))
        equ_height, equ_width = equ_img.shape
        equ_img_clip = equ_img[:, equ_width//2:equ_width]
        _, threshold_image = cv2.threshold(equ_img_clip, 130, 255, cv2.THRESH_BINARY)
        

        cv2.imshow("color", color_image)
        cv2.imshow("equalize", equ_img)

        # cv2.imshow("ellipse", ellipse_drawed_image)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
    
finally:
    #pdb.set_trace()
    pipeline.stop()
