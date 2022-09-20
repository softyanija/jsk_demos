#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "opencv_apps")

(defun adjust-pos-cam1 ()
  (for timer_cam2)

  (setq f 0)
  (while (= f 0)
    (setq socket (send (one-shot-subscribe "/timer_cam1_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
    (setq memory (send (one-shot-subscribe "/timer_cam1_rec/memory/general_contours/restangles" opencv_apps::RotatedRectArrayStamped) :rects))

    (when (and socket memory)
      (progn
        (setq s_x (send 
        
