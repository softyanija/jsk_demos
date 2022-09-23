#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "opencv_apps")

(ros::load-ros-manifest "opencv_apps")

(defun adjust-angle-cam2 ()
  (ros::ros-info (format nil "adjust angle of memory according to cam2 image"))
  (setq angle_diff_tol 3)
  (setq angle_goal -3)
  (setq socket_angle (/ (- (send (send *socket_line_cam2* :pt1) :y) (send (send *socket_line_cam2* :pt2) :y)) (- (send (send *socket_line_cam2* :pt1) :x) (send (send *socket_line_cam2* :pt2) :x))))
      (setq angle (* (/ (atan tilt) pi) 180))
        
  (setq f 0)
  (while (= f 0)
    (setq memory (send (one-shot-subscribe "/timer_cam2_rec/memory/memory_line/line" opencv_apps::LineArrayStamped) :lines))
    (when memory
      (setq memory (elt memory 0))
      (setq memory_tilt (/ (- (send (send memory :pt1) :y) (send (send memory :pt2) :y)) (- (send (send memory :pt1) :x) (send (send memory :pt2) :x))))
      (setq memory_angle (* (/ (atan memory_tilt) pi) 180))
      
      (setq angle_diff (- (- memory_angle socket_angle) angle_goal))
      (ros::ros-info (format nil "angle_diff is ~A" angle_diff))

      (if (> (abs angle_diff) angle_diff_tol)
          (progn
            (if (> (abs angle_diff) 6)
                (setq angle_diff (* 6 (/ angle_diff (abs angle_diff)))))
            (send *pr2* :larm :move-end-rot angle_diff :y)
            (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
            (send *ri* :wait-interpolation))
          (progn
            (setq f 1)
            (ros::ros-info (format nil "end adjusting"))))
        
