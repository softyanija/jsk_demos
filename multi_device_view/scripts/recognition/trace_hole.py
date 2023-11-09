import cv2
import sys
import pdb
import math
import numpy as np

cap = cv2.VideoCapture(8)

prev_centers = []

stage = 0

print("press Space for set before image")

while True:
    ret, frame = cap.read()
    #frame = cv2.resize(frame, (848, 480))

    if not ret:
        break

    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_img = cv2.medianBlur(gray_img, 5)
    
    equ_hist = cv2.equalizeHist(gray_img)
    equ_img = np.hstack((gray_img, equ_hist))
    equ_height, equ_width = equ_img.shape
    equ_img_clip = equ_img[:, equ_width//2:equ_width]
    _, threshold_image = cv2.threshold(equ_img_clip, 90, 255, cv2.THRESH_BINARY)
    #threshold_image = cv2.adaptiveThreshold(equ_img_clip, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    if stage == 0:
        key = cv2.waitKey(1)
        if key == ord(' '):
            gray_img_before = gray_img.copy()
            print("saved before image")
            key_pressed = True
            stage = 1
            print("press Space for set after image")

    if stage == 1:
        
        key = cv2.waitKey(1)
        if key == ord(' '):
            gray_img_after = gray_img.copy()
            
            diff  = cv2.absdiff(gray_img_before, gray_img_after)
            _, diff_thresholded = cv2.threshold(diff, 35, 255, cv2.THRESH_BINARY)
            roi_image = cv2.cvtColor(diff_thresholded.copy(), cv2.COLOR_GRAY2BGR)
            diff_contours, hierarchy = cv2.findContours(diff_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            size_buff = 0
            index_buff = None
            for i, cnt in enumerate(diff_contours):
                x, y, width, height = cv2.boundingRect(cnt)
                if (size_buff < width * height) and (y < gray_img_before.shape[1]//2):
                    size_buff = width * height
                    index_buff = i
            #pdb.set_trace()
            if index_buff is not None:
                x, y, width, height = cv2.boundingRect(diff_contours[index_buff])
                roi_param = ()
                cv2.rectangle(roi_image, (x, y), (x + width, y + height), color=(0, 255, 0), thickness=4)
                print("saved after image")

                key_pressed = True
                stage = 2
            else:
                print("can't find enough diff, try again")

    #_, threshold_image = cv2.threshold(equ_img_clip, 35, 255, cv2.THRESH_BINARY)

    if stage == 2:
        diff  = cv2.absdiff(gray_img_before, gray_img)
        _, diff_thresholded = cv2.threshold(diff, 35, 255, cv2.THRESH_BINARY)
        # diff_thresholded_ex = cv2.cvtColor(diff_thresholded, cv2.COLOR_GRAY2BGR)
        # masked_image = cv2.bitwise_and(frame, diff_thresholded_ex)
        #cv2.imshow("mask", masked_image)
        diff_contours, hierarchy = cv2.findContours(diff_thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        size_buff = 0
        index_buff = None
        for i, cnt in enumerate(diff_contours):
            x, y, width, height = cv2.boundingRect(cnt)
            if (size_buff < width * height) and (y + height//2 < gray_img_before.shape[1]//2):
                size_buff = width * height
                index_buff = i
        #pdb.set_trace()
        if index_buff is not None:
            x, y, width, height = cv2.boundingRect(diff_contours[index_buff])
            roi_param = ()
        mask_roi = np.zeros_like(threshold_image)
        mask_roi = cv2.rectangle(mask_roi, (x, y), (x + width, y + height), 255, cv2.FILLED)
        mask = cv2.bitwise_and(diff_thresholded, mask_roi)
        masked_image = cv2.bitwise_and(threshold_image, mask_roi)
        #masked_image = cv2.bitwise_and(masked_image, mask_roi)
        #mask_image = cv2.bitwise_and(diff_thresholded, mask_roi)
        
        #cv2.imshow("diff_image", diff)
        #cv2.imshow("diff_th", diff_thresholded)
        cv2.imshow("threshold_image", threshold_image)
        cv2.imshow("mask_roi", mask_roi)
        cv2.imshow("masked_image", masked_image)
        
        #ellipse_contours, _ = cv2.findContours(threshold_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        ellipse_contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        ellipse_drawed_image = frame.copy()
        ellipse_list = []
        for i, cnt in enumerate(ellipse_contours):
            if len(cnt) >= 5: 
                ellipse = cv2.fitEllipse(cnt)
                if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                    ellipse_list.append(ellipse)
                    cx = int(ellipse[0][0])
                    cy = int(ellipse[0][1])
                    h = int(ellipse[1][0])
                    w = int(ellipse[1][1])
                    if h * w > 30:
                        ellipse_drawed_image  = cv2.ellipse(ellipse_drawed_image, ellipse, (255,0,0),2)
           
                #ellipse_list = [l[0:2] for l in ellipse_list]

                ellipse_param = np.array([l[0:2] for l in ellipse_list])
                new_centers = []

            # if ellipse_list != []:
            #     ellipse_param = np.array([l[0:2] for l in ellipse_list])
            #     for i, param in enumerate(ellipse_param[:, 0:2]):
            #         center = (param[0][0], param[0][1])

            #         if prev_centers:
            #             # make here
            #             closest_center = min(prev_centers, key=lambda c: np.linalg.norm(np.array(c) - np.array(center)))

            #             if np.linalg.norm(np.array(closest_center) - np.array(center)) < 10:
            #                 cv2.line(frame, (int(closest_center[0]), int(closest_center[1])), (int(center[0]), int(center[1])), (0, 255, 0), 2)
            #                 cv2.ellipse(frame, ellipse_list[i], (0, 255, 0), 2)
            #         #new_centers.append(center)
            #     new_centers.append(center)
            #     prev_centers = new_centers
                # if meet the requirements

                cv2.imshow("ellipse", ellipse_drawed_image)
                cv2.imshow("before", gray_img_before)
                cv2.imshow("after", gray_img_after)
                cv2.imshow("roi", roi_image)
                   

    #cv2.imshow("trace", frame)
    cv2.imshow("equalize", equ_img_clip)
    cv2.imshow("thresholded", threshold_image)
        

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
