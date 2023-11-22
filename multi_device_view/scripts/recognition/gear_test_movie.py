import cv2
import sys
import pdb
import math
import numpy as np
import pyrealsense2 as rs

#cap = cv2.VideoCapture(8)

config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipeline = rs.pipeline()
profile = pipeline.start(config)

try:
    while True:
        
        frames = pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        # gray_img = cv2.cvtColor(frames, cv2.COLOR_BGR2GRAY)
        # gray_img = cv2.medianBlur(gray_img, 3)
    
        # equ_hist = cv2.equalizeHist(gray_img)
        # equ_img = np.hstack((gray_img, equ_hist))
        # equ_height, equ_width = equ_img.shape
        # equ_img_clip = equ_img[:, equ_width//2:equ_width]
        # _, threshold_image = cv2.threshold(equ_img_clip, 130, 255, cv2.THRESH_BINARY)

        # cv2.imshow("equalize", equ_img_clip)
        # cv2.imshow("thresholded", threshold_image)
        cv2.imshow("depth", depth_image)
        cv2.imshow("depth_colormap", depth_colormap)
        # cv2.imshow("ellipse", ellipse_drawed_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break
    
finally:
    pipeline.stop()
