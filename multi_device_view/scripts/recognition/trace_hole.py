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
    _, threshold_image = cv2.threshold(equ_img_clip, 130, 255, cv2.THRESH_BINARY)

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

            if index_buff is not None:
                x, y, width, height = cv2.boundingRect(diff_contours[index_buff])
                roi_param = ()
                cv2.rectangle(roi_image, (x, y), (x + width, y + height), color=(0, 255, 0), thickness=4)
                print("saved after image")

                key_pressed = True
                stage = 2
            else:
                print("can't find enough diff, try again")

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

            
            cv2.imshow("threshold_image", threshold_image)
            cv2.imshow("mask_roi", mask_roi)
            cv2.imshow("masked_image", masked_image)
            cv2.imshow("diff_image", diff)

            ellipse_contours, _ = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
            ellipse_limit_image = frame.copy()
            ellipse_list = []
            x_range_min = 30
            x_range_max = 130
            cv2.rectangle(ellipse_limit_image, (x + x_range_min, y), (x + x_range_max, y + height), (255, 0, 0), 4, cv2.LINE_AA)

            ellipse_max_index = None
            ellipse_max_size = 0
            for i, cnt in enumerate(ellipse_contours):
                if len(cnt) >= 5: 
                    ellipse = cv2.fitEllipse(cnt)
                    if (not math.isnan(ellipse[0][0])) and (not math.isnan(ellipse[0][1])) and (not math.isnan(ellipse[1][0])) and (not math.isnan(ellipse[1][1])):
                        ellipse_list.append(ellipse)
                        cx = int(ellipse[0][0])
                        cy = int(ellipse[0][1])
                        h = int(ellipse[1][0])
                        w = int(ellipse[1][1])
                        if (h * w > 300) and (h * w < 2000) and (cx > x + x_range_min) and(cx < x + x_range_max):
                            if h * w > ellipse_max_size:
                                ellipse_max_size = h * w
                                ellipse_max_index = i
                            #ellipse_limit_image  = cv2.ellipse(ellipse_limit_image, ellipse, (255,0,0),2)
                            print(ellipse)

                    ellipse_param = np.array([l[0:2] for l in ellipse_list])
                    new_centers = []
                if ellipse_max_index is not None:
                    ellipse = cv2.fitEllipse(ellipse_contours[ellipse_max_index])
                    ellipse_limit_image  = cv2.ellipse(ellipse_limit_image, ellipse, (255,0,0), 2)
                    
            cv2.imshow("ellipse", ellipse_limit_image)
            cv2.imshow("before", gray_img_before)
            cv2.imshow("after", gray_img_after)
            cv2.imshow("roi", roi_image)

    cv2.imshow("equalize", equ_img_clip)
    cv2.imshow("thresholded", threshold_image)
        

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
