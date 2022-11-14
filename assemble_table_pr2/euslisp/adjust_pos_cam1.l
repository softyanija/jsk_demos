#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "opencv_apps")

(ros::load-ros-manifest "opencv_apps")

(defun get-cam1-pos-param ()
  (ros::ros-info (format nil "measuring cam1-pos-param"))
  (get-cam1-memory)
  (setq x_a (send (send *memory_cam1* :center) :x))
  (setq y_a (+ (send (send *memory_cam1* :center) :y) (/ (max (send (send *memory_cam1* :size) :height) (send (send *memory_cam1* :size) :width)) 2)))
  
  (send *pr2* :larm :move-end-pos #f(-5 -5 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (unix:sleep 5)
  (get-cam1-memory)
  (setq x_b (send (send *memory_cam1* :center) :x))
  (setq y_b (+ (send (send *memory_cam1* :center) :y) (/ (max (send (send *memory_cam1* :size) :height) (send (send *memory_cam1* :size) :width)) 2)))

  (setq x_rate (/ (- x_b x_a) -5))
  (setq y_rate (/ (- y_b y_a) -5))
  (ros::ros-info (format nil "x_rate:~A y_rate:~A" x_rate y_rate))

  (send *pr2* :larm :move-end-pos #f(5 5 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation))

(defun adjust-pos-cam1 ()
  (ros::ros-info (format nil "adjusting pos of memory according to cam1"))
  (setq x_goal 0)
  (setq y_goal -10)

  (setq f 0)
  (while (= f 0)
    (setq socket_cand (send (one-shot-subscribe "/timer_cam1_rec/socket/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
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
        )))
  (setq socket_x (send (send socket :center) :x))
  (setq socket_y (send (send socket :center) :y))

  (setq f 0)
  (while (= f 0)
    (get-cam1-memory)
    (setq f 0)
    (setq memory_x (send (send *memory_cam1* :center) :x))
    (setq memory_y (+ (send (send *memory_cam1* :center) :y) (/ (max (send (send *memory_cam1* :size) :height) (send (send *memory_cam1* :size) :width)) 2)))

    (setq x_diff (- memory_x socket_x))
    (setq y_diff (- memory_y socket_y))
    (setq x_corr (- x_diff x_goal))
    (setq y_corr (- y_diff y_goal))

    (ros::ros-info (format nil "x_corr is ~A" x_corr))
    (ros::ros-info (format nil "y_corr is ~A" y_corr))

    (if (and (< (abs x_corr) 2) (< (abs y_corr) 5))
        (progn
          (ros::ros-info "adjust-pos finished")
          (setq f 1))
        (progn
          (ros::ros-info "move hand")
          (setq y_delta (/ (- 0 x_corr) x_rate))
          (setq x_delta (/ (- 0 y_corr) y_rate))
          (send *pr2* :larm :move-end-pos (float-vector x_delta y_delta 0))
          (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
          (send *ri* :wait-interpolation)
          (unix:sleep 3)
          )
        )
    )
)
        
