#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "dynamic_reconfigure")

(ros::load-ros-manifest "opencv_apps")

(ros::roseus "diff_test")

(defun background-init ()
  (let ((srv-name (format nil "/timer_cam2_rec/background_subtraction/set_parameters")) (req (instance dynamic_reconfigure::ReconfigureRequest :init)))
	(send req :config :doubles
		  (list (instance dynamic_reconfigure::DoubleParameter :init
						  :name "false" :value 3)))
	(if (ros::wait-for-service srv-name 0)
		(ros::service-call srv-name req)))
  t)

;; (:change-inflation-range
;;    (&optional (range 0.2)
;;     &key (node-name "/move_base_node")
;;          (costmap-name "local_costmap")
;;          (inflation-name "inflation"))
;;    "Changes inflation range of local costmap for obstacle avoidance."
;;    (let ((srv-name (format nil "~A/~A/~A/set_parameters" node-name costmap-name inflation-name))
;;          (req (instance dynamic_reconfigure::ReconfigureRequest :init)))
;;      (send req :config :doubles
;;            (list (instance dynamic_reconfigure::DoubleParameter :init
;;                            :name "inflation_radius" :value range)))
;;      (if (ros::wait-for-service srv-name 0)
;;          (ros::service-call srv-name req)))
;;    t)
