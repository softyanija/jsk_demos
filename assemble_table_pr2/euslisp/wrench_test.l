;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(setq *null-wrench* #f(0 0 0))
(setq push_z_lim 1.5)

(send *pr2* :larm :inverse-kinematics *larm-pre-apply*)
(send *ri* :angle-vector (send *pr2* :angle-vector))
(send *ri* :wait-interpolation)

(setq i 0)
(while (< i 15)
  (setq force_z_p (send (send (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench) :force) :z))
  (setq force_z (* -1 force_z_p))
  (ros::ros-info (format nil "counter force is ~A" force_z))
  (if (> force_z push_z_lim)
      (progn
	(ros::ros-info "detect contacting")
	(return))
      (progn
	(ros::ros-info "still hanging")
	(send *pr2* :larm :move-end-pos #f(0 0 3))
	(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
	(send *ri* :wait-interpolation)

	(setq stroke (* 3 i))
	(setq i (+ i 1))
	(ros::ros-info (format nil "stroke is ~A" stroke)))))

;;chech if botl and nat is -- directly

(setq base-coords (send *pr2* :larm :end-coords))

(setq move-radius 2.5)
(setq i 1)
(setq n 0)
(stq f 0)
(setq apply_lim_z 1)

(while (and (< i 4) (= f 0))
  (setq n 0)
  (while (and (< n 9) (= f 0))
    (send *pr2* :larm :inverse-kinematics base-coords)
    (setq move-vector (scale i (scale move-radius (float-vector (sin (* n (/ pi 4))) (cos (* n (/ pi 4))) 0))))
    (send *pr2* :larm :move-end-pos move-vector)
    (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
    (send *ri* :wait-interpolation)
    (ros::ros-info (format nil "now n:~A i:~A" n i))

    (setq force_z_p (send (send (send (one-shot-subscribe "/left_endeffector/wrench" geometry_msgs::WrenchStamped) :wrench) :force) :z))
    (setq force_z (* -1 force_z_p))
    (if (< force_z apply_lim_z)
	(progn
	  (setq f 1)
	  (ros::ros-info "success applying")
	  (return)))
    (setq n (+ n 1))
    (ros::ros-info (format nil "n = ~A" n)))
  (setq i (+ i 1)))
