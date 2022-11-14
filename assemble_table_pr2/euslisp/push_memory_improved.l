#!/usr/bin/env roseus
(ros::roseus-add-msgs "opencv_apps")
(ros::load-ros-manifest "opencv_apps")

(defun push_memory ()
  (setq size_lim_cam2 30)
  (setq size_lim_cam1 30)
  
  (larm-move-end-pos-worldcoords #f(0 10 14))
  (send *pr2* :larm :move-end-rot 90 :x)
  (send *pr2* :larm :move-end-rot -1 :z)
  (send *pr2* :larm :move-end-rot 1 :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :move-gripper :larm 0.030)

  (larm-move-end-pos-worldcoords #f(0 20 0))

  (get-cam2-socket)
  (setq socket_x (send (send *socket_cam2* :center) :x))
  (setq socket_y (send (send *socket_cam2* :center) :y))
  (setq rate_x 4)
  (setq rate_y 4)
  (setq goal_x -30)
  (setq goal_y -105)

  (setq f 0)
  (while (= f 0)
    (setq hand_pos (send (one-shot-subscribe "/timer_cam2_rec/hand_pos" geometry_msgs::PoseArray) :poses))
    (when hand_pos
      (setq hand_x (send (send (elt hand_pos 0) :position) :x))
      (setq hand_y (send (send (elt hand_pos 0) :position) :y))
      (setq diff_x (- (- hand_x socket_x) goal_x))
      (setq diff_y (- (- hand_y socket_y) goal_y))
      (if (not (and (< (abs diff_x) 3) (< (abs diff_y) 5)))
          (setq move_x (- 0 (/ diff_y rate_y)))
          (setq move_y (/ diff_x rate_x))

          (send *pr2* :larm :move-end-pos (float-vector move_x move_y 0))
          (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
          (send *ri* :wait-interpolation)
          (setq rate_x (* rate_x 1.2))
          (setq rate_y (* rate_y 1.2))
          (unix:sleep 1))))

  (setq f 0)
  (while (f = 0)
    (setq f 1)
    (send *pr2* :larm :move-end-rot 5 :z)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -40))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    
    (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) 40))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
        (send *pr2* :larm :move-end-rot -5 :z)
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
        (if (> size_max size_lim_cam2)
            (progn
              (ros::ros-info "memory of cam2 isn't locked")
              (setq f 0))
            (ros::ros-info "memory of cam2 side is locked"))
            )))

  (larm-move-end-pos-worldcoords #f(0 -60 0))

  (setq f 0)
  (while (f = 0)
    (setq f 1)
    (send *pr2* :larm :move-end-rot -5 :z)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) -40))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)
    
    (send *pr2* :torso_lift_joint :joint-angle (+ (send *pr2* :torso_lift_joint :joint-angle) 40))
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (send *pr2* :larm :move-end-rot 5 :z)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
    (send *ri* :wait-interpolation)

    (setq rects (send (one-shot-subscribe "/timer_cam1_rec/lever/general_contours/rectangles" opencv_apps::RotatedRectArrayStamped) :rects))
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
        (if (> size_max size_lim_cam1)
            (progn
              (ros::ros-info "memory of cam1 isn't locked")
              (setq f 0))
            (ros::ros-info "memory of cam1 side is locked"))
        ))
    )

  (larm-move-end-pos-worldcoords #f(0 -30 100)))
