;;(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(load "package://pr2eus/pr2-interface.l")
(load "package://assemble_table_pr2/euslisp/parameters.l")
(load "package://assemble_table_pr2/euslisp/utils.l")
(load "package://assemble_table_pr2/euslisp/pr2-manequine.l")
(pr2-init)
(objects (list *pr2*))

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::load-ros-manifest "geometry_msgs")

(ros::roseus "use_box_test")
(setq *null-wrench* #f(0 0 0))
(setq push_z_lim 1.3)

(send *pr2* :angle-vector *check-table*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(setq stamp (ros::time-now))
(when (send *tfl* :wait-for-transform "base_link" "leg" stamp 5)
  (setq leg-origin (send *tfl* :lookup-transform "base_link" "leg" stamp)))

(setq base-to-leg (send (send leg-origin :copy-worldcoords) :transform *larm-pickleg*))

(send *pr2* :larm :inverse-kinematics base-to-leg)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :move-gripper :larm 0.050)
(unix:sleep 1)

(send *pr2* :larm :move-end-pos #f(70 0 0))
(send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm)
(unix:sleep 2)

(send *pr2* :larm :inverse-kinematics base-to-leg)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *pr2* :larm :inverse-kinematics *larm-raise-leg*)
(send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
(send *ri* :wait-interpolation)


#|
(send *pr2* :angle-vector (send *ri* :state :potentio-vector))
(setq btgp (send *pr2* :larm :end-coords))
(setq param (send (send (send leg-origin :inverse-transformation) :copy-worldcoords) :transform (send btgp :copy-worldcoords)))
(setq test (send (send base-to-marker-origin :copy-worldcoords) :transform param))
(objects (list *robot* base-to-marker-origin test))
|#
