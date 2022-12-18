#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(defun set-memory ()

  (larm-move-end-pos-worldcoords #f(0 3 -6))
  (larm-move-end-pos-worldcoords #f(2 0 0))
  (larm-move-end-pos-worldcoords #f(-2 0 0))
  (larm-move-end-pos-worldcoords #f(0 -2 0))
  (larm-move-end-pos-worldcoords #f(0 0 -5))
  (larm-move-end-pos-worldcoords #f(0 -3 0))
  
  (send *pr2* :larm :move-end-rot -1.5 :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :larm 0.015)
  (unix:sleep 1)
  
  (setq result 4)
  (setq stroke 1)
  (while (not (= result 0))
    
    (setq result (judge-set-memory-cam2))
    (case result
      (0
       (progn
         (ros::ros-info "success memory setting")
         ))
      (1
       (progn
         (ros::ros-info "memory is rear, adjusting memory")
         (larm-move-end-pos-worldcoords #f(-2 0 7))
         (larm-move-end-pos-worldcoords #f(0 0 -7))
         (send *pr2* :larm :move-end-rot 12.0 :y)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
         (send *ri* :wait-interpolation))
         (send *ri* :start-grasp :larm)
         (larm-move-end-pos-worldcoords #f(0 -1 7))
         (larm-move-end-pos-worldcoords #f(5 0 0))
         (send *ri* :move-gripper :larm 0.015)
         (unix:sleep 1)
         (send *pr2* :larm :move-end-rot -12.0 :y)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
         (send *ri* :wait-interpolation)
         ))
      (2
       (progn
         (ros::ros-info "memory is front, adjusting memory")
         (larm-move-end-pos-worldcoords #f(2 0 7))
         (larm-move-end-pos-worldcoords #f(0 0 -7))
         (send *pr2* :larm :move-end-rot 12.0 :y)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
         (send *ri* :wait-interpolation))
         (send *ri* :start-grasp :larm)
         (larm-move-end-pos-worldcoords #f(0 -2 7))
         (larm-move-end-pos-worldcoords #f(-5 0 0))
         (send *ri* :move-gripper :larm 0.015)
         (unix:sleep 1)
         (send *pr2* :larm :move-end-rot -12.0 :y)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 1500)
         (send *ri* :wait-interpolation)
         ))
      (3
       (progn
         (while (= (judge-set-memory-cam2) 3)
           (setq stroke (* -1 (/ stroke (abs stroke)) (+ 1 (abs stroke))))
           (larm-move-end-pos-worldcoords (float-vector stroke 0 0)))
         )
       )
       )))

