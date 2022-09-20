#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "opencv_apps")

(ros::load-ros-manifest "opencv_apps")

(defun adjust-pos-cam2 ()
  (for timer_cam2)

  (setq f 0)
  (while (= f 0)
    (setq socket (send (one-shot-subscribe "/timer_cam2_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
    (setq memory (send (one-shot-subscribe "/timer_cam2_rec/memory_edge" geometry_msgs::PoseArray) :poses))

    (when (and socket memory)
      (progn
        (setq x_0 (send (send (elt memory 0) :position) :x))
        (setq y_0 (send (send (elt memory 0) :position) :y))
        (setq x_1 (send (send (elt memory 1) :position) :x))
        (setq y_1 (send (send (elt memory 1) :position) :y))
        (setq x_2 (send (send (elt memory 2) :position) :x))
        (setq y_2 (send (send (elt memory 2) :position) :y))
        (setq x_3 (send (send (elt memory 3) :position) :x))
        (setq y_3 (send (send (elt memory 3) :position) :y))
        (setq e_0 (+ x_0 y_0))
        (setq e_1 (+ x_1 y_1))
        (setq e_2 (+ x_2 y_2))
        (setq e_3 (+ x_3 y_3))
        (setq memory_edge (max e_0 e_1 e_2 e_3))

        (setq l (length socket)) 
        
        
        (setq socket_x (send (send (elt socket 0) :center) :x))
        
