import cv2
import sys
import pdb
import math
import numpy as np

cap = cv2.VideoCapture(8)

prev_centers = []

while True:
    ret, frame = cap.read()
    #frame = cv2.resize(frame, (848, 480))

    if not ret:
        break

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.medianBlur(gray_img, 3)
    
    equ_hist = cv2.equalizeHist(gray_img)
    equ_img = np.hstack((gray_img, equ_hist))
    equ_height, equ_width = equ_img.shape
    equ_img_clip = equ_img[:, equ_width//2:equ_width]

    _, threshold_image = cv2.threshold(equ_img_clip, 25, 255, cv2.THRESH_BINARY)
    
    ellipse_contours, _ = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    ellipse_drawed_image = frame.copy()
    ellipse_list = []
    for i, cnt in enumerate(ellipse_contours):
        if len(cnt) >= 5: 
            ellipse = cv2.fitEllipse(cnt)
            if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                ellipse_list.append(ellipse)
            try:
                if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                    cx = int(ellipse[0][0])
                    cy = int(ellipse[0][1])
                    ellipse_drawed_image  = cv2.ellipse(ellipse_drawed_image, ellipse, (255,0,0),2)
            except Exception as e:
                pdb.set_trace()

           
    #ellipse_list = [l[0:2] for l in ellipse_list]
    if ellipse_list != []:
        #ellipse_centers = np.uint16(np.around(ellipse_param)
        ellipse_param = np.array([l[0:2] for l in ellipse_list])
        new_centers = []

        
        for i, param in enumerate(ellipse_param[:, 0:2]):
            center = (param[0][0], param[0][1])

            if prev_centers:
                closest_center = min(prev_centers, key=lambda c: np.linalg.norm(np.array(c) - np.array(center)))
                try:
                    cv2.line(frame, (int(closest_center[0]), int(closest_center[1])), (int(center[0]), int(center[1])), (0, 255, 0), 2)
                except Exception as e:
                    pdb.set_trace()
            new_centers.append(center)
            #pdb.set_trace()
            if (not math.isnan(ellipse_list[i][0][0])) and (not math.isnan(ellipse_list[i][0][1])) and (not math.isnan(ellipse_list[i][1][0])) and (not math.isnan(ellipse_list[i][1][1])):
                cv2.ellipse(frame, ellipse_list[i], (0, 255, 0), 2)

        prev_centers = new_centers
                       


    cv2.imshow("detected ellipse", frame)
    cv2.imshow("equalize", equ_img_clip)
    cv2.imshow("thresholded", threshold_image)
    cv2.imshow("ellipse", ellipse_drawed_image)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

