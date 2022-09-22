#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "opencv_apps")

(ros::load-ros-manifest "opencv_apps")

(defun adjust-pos-cam1 ()
  (ros::ros-info (format nil "adjusting pos of memory according to cam1"))
  (setq angle_diff_tol 5)

  (setq f_a 0)
  (while (= f_a 0)
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
            (setq f_a 1))
        )))
  (setq socket_x (send (send socket :center) :x))
  (setq socket_y (- (send (send socket :center) :y) (/ (send (send socket :size) :height) 2)))
  ;;ロボットの動作量と画像内での移動量の対応を取る
  (setq f_b 0)
  (while (= f_b 0)
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

        (setq memory_h (max (send (send memory :size) :height) (send (send memory :size) :width)))
        (setq memory_x (send (send memory :center) :x))
        (setq memory_y (+ (send (send memory :center) :y) (/ memory_h 2)))
        
        
        (ros::ros-info (format nil "angle_diff is ~A" angle_diff))

        (if (> (abs angle_diff) 3)
            (progn
              (if (> (abs angle_diff) 10)
                  (setq angle_diff (* 10 (/ angle_diff (abs angle_diff)))))
              (ros::ros-info (format nil "hoge"))
              (send *pr2* :larm :move-end-rot angle_diff :z)
              (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
              (send *ri* :wait-interpolation))
            (progn
              (setq f_b 1)
              (ros::ros-info (format nil "end adjusting"))))
        (unix:sleep 1)
        ))))

    
        