(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(setq *null-wrench* #f(0 0 0))
(setq wrench (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench))  

(#SET TO INSERT POS)

(setq wrench (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench))
(while (< i 15)
  (if (> force_z z_lim)
      (progn
	(ros::ros-info "detect contacting")
	(return))
      (progn
	(ros::ros-info "still hanging")
	(send *pr2* :move-end-pos #f())
	(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
	(send *ri* :wait-interpolation)
	(setq i (+ i 1))
	(ros::ros-info (format nil "stroke is ~A" stroke)))))

;;chech if botl and nat is -- directly

(setq base-coords (send *pr2* :larm :end-coords))

(setq move-radius 2.5) ;(mm)
(setq i 1)
(setq n 0)

(while (< i 3)
  (setq n 0)
  (while (< n 9)
    (send *pr2* :larm :inverse-kinematics base-coords)
    (setq move-vector (scale i (scale radius (float-vector (sin (* n (/ pi 4))) (cos (* n (/ pi 4))) 0))))
    (send *pr2* :larm :move-end-pos move-vector)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (ros::ros-info (format nil "now n:~A i:~A" n i))

    (setq wrench (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench))
    (setq force_z (send wrench :z))
    (if (< force_z lim_z)
	(progn
	  (ros::ros-info "success applying")
	  (return)))
    (setq n (+ n 1)))
  
  (setq i (+ i 1)))
