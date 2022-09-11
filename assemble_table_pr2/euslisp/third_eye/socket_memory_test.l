#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::load-ros-manifest "opencv_apps")

(ros::roseus "socket_test")

(setq rects (send (one-shot-subscribe "/timer_cam_1_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
(when rects
  (progn
    (setq l (length rects))
    (setq i 0)
    (setq max_i 0)
    (setq size_max 0)
    (setq size_buf 0)
    (setq fx 0)
    (setq fy 0)
    

    (while (< i l)
      (setq g_x (send (send (elt rects i) :center) :x))
      (setq g_y (send (send (elt rects i) :center) :y))
      (setq l_x (send (send (elt rects i) :size) :width))
      (setq l_y (send (send (elt rects i) :size) :height))
	  (setq angle (send (elt rects i) :angle))
      (if (and t (> g_x 20) (< g_x 300) (> g_y 30) (< g_y 210) (> l_x l_y))
		  (progn
			(setq size_buf (* (send (send (elt rects i) :size) :width) (send (send (elt rects i) :size) :height)))
			(if (> size_buf size_max)
				(progn
				  (setq size_max size_buf)
				  (setq max_i i)
				  (setq top g_x)
				  (setq top_y (- g_y (/ l_y 2)))))
			(ros::ros-info (format nil "index:~A, angle:~A, size:~A, l_x:~A, l_y:~A g_x:~A, g_y:~A" i angle size_buf l_x l_y g_x g_y))))
      (setq i (+ i 1)))))
(setq i 0)



