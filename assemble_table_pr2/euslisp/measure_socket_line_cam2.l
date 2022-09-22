#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "opencv_apps")

(ros::load-ros-manifest "opencv_apps")

(defun measure-socket-line-cam2 ()
  (ros::ros-info (format nil "measuring socket line"))
  (setq f 0)
  (while (= f 0)
    (setq socket_cand (send (one-shot-subscribe "/timer_cam2_rec/socket/hough_lines/lines" opencv_apps::LineArrayStamped) :lines))
    (when socket_cand
      (progn
        (setq l (length socket_cand))
        (setq i 0)
        (setq max_i 0)
        (setq center_y_max 0)
        (while (< i l)
          (setq line (elt socket_cand i))
          (setq center_y (/ (+ (send (send line :pt1) :y) (send (send line :pt2) :y)) 2))
          (ros::ros-info (format nil "center_y:~A" center_y))
           
          (if (> center_y center_y_max)
              (progn
                (setq center_y_max center_y)
                (setq max_i i)))
          (incf i))
        (setq line (elt socket_cand max_i))
        (setq center_y (/ (+ (send (send line :pt1) :y) (send (send line :pt2) :y)) 2))
        (if (and (> center_y 150) (< (* 2 (abs (- (send (send line :pt1) :y) (send (send line :pt2) :y)))) (abs (- (send (send line :pt1) :x) (send (send line :pt2) :x)))))
            (progn
              (setq *socket_line_cam2* line)
              (setq f 1)))))))
            
