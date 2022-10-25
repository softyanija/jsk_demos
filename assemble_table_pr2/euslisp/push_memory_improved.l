#!/usr/bin/env roseus
(ros::roseus-add-msgs "opencv_apps")
(ros::load-ros-manifest "opencv_apps")

(defun push_memory ()
  (setq size_lim 30)
  
  (larm-move-end-pos-worldcoords #f(0 0 18))
  (send *pr2* :larm :move-end-rot 90 :x)
  (send *pr2* :larm :move-end-rot -1 :z)
  (send *pr2* :larm :move-end-rot 1 :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  

  (larm-move-end-pos-worldcoords #f(0 45 0))
  (send *ri* :move-gripper :larm 0.030)

  (setq f 0)
  (while (f = 0)
    (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -20))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot -15 :z)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *pr2* :larm :move-end-rot 15 :z)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) 20))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    ;;check if lever was locked
    (setq rects (send (one-shot-subscribe "/timer_cam2_rec/lever/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
    (when rects
      (progn
        (setq l (length rects))
        (setq i 0)
        (setq size_max 0)
        (while (< i l)
          (setq rect (elt rects i))
          (setq size_buf (* (send (send rect :size) :width) (send (send rect :size) :height)))
          (if (> size_buf size_max)
              (progn
                (setq size_max size_buf)
                (setq max_i i)))
          (incf i))
        (if (> size_max size_lim)
            (progn
              (ros::ros-info "memory of cam2 side is locked")
              (setq f 1))
            (ros::ros-info "memory of cam2 isn't locked")))))
    

  (larm-move-end-pos-worldcoords #f(0 -87 0))

  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :larm :move-end-rot -15 :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *pr2* :larm :move-end-rot 15 :z)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) 20))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (larm-move-end-pos-worldcoords #f(0 -30 100))
