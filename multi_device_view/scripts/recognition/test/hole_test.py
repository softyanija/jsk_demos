import cv2
import sys
import pdb
import math
import numpy as np

cap = cv2.VideoCapture(8)

while True:
    ret, frame = cap.read()

    if not ret:
        break

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.medianBlur(gray_img, 5)
    
    equ_hist = cv2.equalizeHist(gray_img)
    equ_img = np.hstack((gray_img, equ_hist))
    equ_height, equ_width = equ_img.shape
    equ_img_clip = equ_img[:, equ_width//2:equ_width]
    _, threshold_image = cv2.threshold(equ_img_clip, 130, 255, cv2.THRESH_BINARY)


    ellipse_contours, hierarcy = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    ellipse_drawed_image = frame.copy()
    ellipse_list = []
    for i, cnt in enumerate(ellipse_contours):
        if (len(cnt) >= 5): 
            ellipse = cv2.fitEllipse(cnt)
            if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                ellipse_list.append(ellipse)
                cx = int(ellipse[0][0])
                cy = int(ellipse[0][1])
                h = int(ellipse[1][0])
                w = int(ellipse[1][1])
                if (h * w > 300) and (h * w < 2000):
                    ellipse_drawed_image  = cv2.ellipse(ellipse_drawed_image, ellipse, (255,0,0),2)
                    ellipse_drawed_image = cv2.putText(ellipse_drawed_image, str(h * w), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,80,255), 1,cv2.LINE_AA)

                    """
    ellipse_drawed_image = frame.copy()
    circles = cv2.HoughCircles(
        equ_img_clip, 
        cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        param1=50, param2=30, minRadius=10, maxRadius=100
    )

    # 検出された楕円を描画
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(ellipse_drawed_image, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            cv2.circle(ellipse_drawed_image, (i[0], i[1]), 2, (0, 0, 255), 3)
    """
    

    cv2.imshow("equalize", equ_img_clip)
    cv2.imshow("thresholded", threshold_image)
    cv2.imshow("ellipse", ellipse_drawed_image)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
