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
  (send *pr2* :larm :move-end-rot -3 :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :larm 0.015)
  (unix:sleep 1)
  
  (setq result 4)
  (while (not (= result 0))
    
    (setq result (judge-set-memory-cam2))
    (case result
      (0
       (progn
         (ros::ros-info "success memory setting")
         )
      (1
       (progn
         (ros::ros-info "adjusting memory")
         )
      (2
       (progn
         (ros::ros-info "adjusting memory")
         )
      (3
       (progn
         (while (= (judge-set-memory-cam2) 3)
           (larm-move-end-pos-worldcoords #f(2 0 0)))
         )
       )
       ))))

