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

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

        mask = np.copy(depth_image)
        th_range_min = 700
        th_range_max = 1000
        mask[mask < th_range_min] = 0
        mask[mask <= th_range_max] = 255
        mask[mask > th_range_max] = 0
        mask = mask.astype(np.uint8)

        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        #pdb.set_trace()
        #_, mask = cv2.threshold(depth_image, 50, 255, cv2.THRESH_BINARY)
        # mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # pdb.set_trace()
        cliped_image = cv2.bitwise_and(color_image, mask_color)

        # cv2.imshow("equalize", equ_img_clip)

        cv2.imshow("color", color_image)
        cv2.imshow("depth", depth_image)
        cv2.imshow("thresholded", mask)
        cv2.imshow("mask_color", mask_color)
        cv2.imshow("depth_colormap", depth_colormap)
        cv2.imshow("cliped_colormap", cliped_image)
        # cv2.imshow("ellipse", ellipse_drawed_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break
    
finally:
    pipeline.stop()
