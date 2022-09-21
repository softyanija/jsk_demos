#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "opencv_apps")

(ros::load-ros-manifest "opencv_apps")

(defun adjust-angle-cam1 ()
  (ros::ros-info (format nil "adjusting angle of memory according to cam1"))
  (setq angle_diff_tol 5)

  (setq f 0)
  (while (= f 0)
    (setq socket_cand (send (one-shot-subscribe "/timer_cam1_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
  
  (setq f 0)
  (while (= f 0)
    (when socket_cand
      (progn
        (setq l (length socket_cand))
        (setq i 0)
        (setq size_max 0)
        (while (< i l)
          (setq rect (elt socket_cand i))
          (setq size_buf (* (send (send rect :size) :width) (send (send rect :size) :height)))
          (if (> size_buf size_max)
              (progn
                (setq size_max size_buf)
                (setq max_i i)))
          (incf i))
        (setq socket (elt socket_cand max_i))
        (if (> size_max 150)
            (setq f 1))
        ))))  
  
  (setq f 0)
  (while (= f 0)
    (setq memory_cand (send (one-shot-subscribe "/timer_cam1_rec/memory/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
    (when memory_cand
      (progn
        (setq l (length memory_cand))
        (setq i 0)
        (setq size_max 0)
        (setq lim_y 50)
        (while (< i l)
          (setq rect (elt memory_cand i))
          (if (< lim_y (send (send rect :center) :y))
              (progn
                (setq size_buf (* (send (send rect :size) :width) (send (send rect :size) :height)))
                (if (> size_buf size_max)
                    (progn
                      (setq size_max size_buf)
                      (setq max_i i)))))
          (incf i))
        (setq memory (elt memory_cand max_i))
        
        (setq memory_angle (send memory :angle))
        (if (< memory_angle -60)
            (setq memory_angle (+ memory_angle 90)))
        
        (setq socket_angle (send socket :angle))
        (if (< socket_angle -60)
            (setq socket_angle (+ socket_angle 90)))

        (setq angle_diff (- memory_angle socket_angle))
        (ros::ros-info (format nil "angle_diff is ~A" angle_diff))

        (if (> (abs angle_diff) 3)
            (progn
              (if (> (abs angle_diff) 10)
                  (setq angle_diff (* 10 (/ angle_diff (abs angle_diff)))))
              (send *pr2* :larm :move-end-rot angle_diff :z)
              (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
              (send *ri* :wait-interpolation))
            (progn
              (setq f 1)
              (ros::ros-info (format nil "end adjusting"))))
        ))))

    
        
