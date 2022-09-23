#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(defun set-memory ()

  (larm-move-end-pos-worldcoords #f(0 0 -6))
  (larm-move-end-pos-worldcoords #f(0 -2 0))

  (larm-move-end-pos-worldcoords #f(0 0 -5))
  (send *pr2* :larm :move-end-rot -3 :y)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (send *ri* :move-gripper :larm 0.015)
  (unix:sleep 1)
  
  (setq f 0)
  (while (not (= f 0))
    
    ();ninshiki 0:success 1:oku 2:temae 3:ue
    (case f
      (0
       (progn
         (ros::ros-info "success memory setting")
         ))
      (1
       (progn
         (larm-move-end-pos-worldcoords #f(0 0 -3))
         (send *ri* :start-grasp :larm)
         (send *pr2* :larm :move-end-pos #f(0 2.5 0))
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (send *pr2* :larm :move-end-rot 3 :y)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (larm-move-end-pos-worldcoords #f(0 0 2))
         (send *pr2* :larm :move-end-pos #f(0 2.5 0))
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (send *pr2* :larm :move-end-rot 2 :x)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (larm-move-end-pos-worldcoords #f(0 -2 0))
         (send *ri* :move-gripper :larm 0.020)
         ))
      (2
       (progn
         (larm-move-end-pos-worldcoords #f(0 0 -12))
         (send *ri* :start-grasp :larm)
         (larm-move-end-pos-worldcoords #f(0 0 3))
         (send *pr2* :larm :move-end-rot 3 :y)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (send *pr2* :larm :move-end-pos #f(0 -5 0))
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (send *pr2* :larm :move-end-rot -2 :x)
         (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
         (send *ri* :wait-interpolation)
         (larm-move-end-pos-worldcoords #f(0 -2 0))
         (send *ri* :move-gripper :larm 0.015)
         ))
      (3
       (progn
         (send *ri* :start-grasp :larm)
         (larm-move-end-pos-worldcoords #f(0 0 -5))
         (if (check-set-memory-cam2)
             (setq f 0)
             (progn
               (setq set 0)
               (larm-move-end-pos-worldcoords #f(0 0 3))
               (send *pr2* :larm :move-end-pos #f(0 -1 0))
               (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
               (send *ri* :wait-interpolation)
               (larm-move-end-pos-worldcoords #f(0 0 -3))
               (if (check-set-memory-cam2)
                   (progn
                     (setq set 1)
                     (setq f 0)))

               (if (= set 0)
                   (progn
                     (larm-move-end-pos-worldcoords #f(0 0 3))
                     (send *pr2* :larm :move-end-pos #f(0 2 0))
                     (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
                     (send *ri* :wait-interpolation)
                     (larm-move-end-pos-worldcoords #f(0 0 -3))
                     (if (check-set-memory-cam2)
                         (progn
                           (setq set 1)
                           (setq f 0)))))
               (if (= set 0)
                   (progn
                     (larm-move-end-pos-worldcoords #f(0 0 3))
                     (send *pr2* :larm :move-end-pos #f(0 -3 0))
                     (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
                     (send *ri* :wait-interpolation)
                     (larm-move-end-pos-worldcoords #f(0 0 -3))
                     (if (check-set-memory-cam2)
                         (progn
                           (setq set 1)
                           (setq f 0)))))
               (if (= set 0)
                   (progn
                     (larm-move-end-pos-worldcoords #f(0 0 3))
                     (send *pr2* :larm :move-end-pos #f(0 4 0))
                     (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
                     (send *ri* :wait-interpolation)
                     (larm-move-end-pos-worldcoords #f(0 0 -3))
                     (if (check-set-memory-cam2)
                         (progn
                           (setq set 1)
                           (setq f 0)))))
               (if (= set 0)
                   (progn
                     (larm-move-end-pos-worldcoords #f(0 0 3))
                     (send *pr2* :larm :move-end-pos #f(0 -5 0))
                     (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
                     (send *ri* :wait-interpolation)
                     (larm-move-end-pos-worldcoords #f(0 0 -3))
                     (if (check-set-memory-cam2)
                         (progn
                           (setq set 1)
                           (setq f 0)))))
               (if (= set 0)
                   (progn
                     (larm-move-end-pos-worldcoords #f(0 0 3))
                     (send *pr2* :larm :move-end-pos #f(0 6 0))
                     (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
                     (send *ri* :wait-interpolation)
                     (larm-move-end-pos-worldcoords #f(0 0 -3))
                     (if (check-set-memory-cam2)
                         (progn
                           (setq set 1)
                           (setq f 0)))))))
         (send *ri* :move-gripper :larm 0.015))
       ))))

