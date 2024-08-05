;; manequine
(ros::load-ros-manifest "pr2_mechanism_msgs")
;; (ros::wait-for-service "/pr2_controller_manager/switch_controller")

(defun manequine-mode (&key (arm (list "rarm")) (start t))
  (let ((req (instance pr2_mechanism_msgs::SwitchControllerRequest :init))
	(normal-controllers (list))
	(loose-controllers (list))
	)
    (dolist (e arm)
      (if (equal e "rarm")
	  (progn
	    (setq normal-controllers (append normal-controllers (list "r_arm_controller")))
	    (setq loose-controllers (append loose-controllers (list "r_arm_controller_loose")))
	    )
	)
      (if (equal e "larm")
	  (progn
	    (setq normal-controllers (append normal-controllers (list "l_arm_controller")))
	    (setq loose-controllers (append loose-controllers (list "l_arm_controller_loose")))
	    )
	))
    (if start
	(progn
	  (send req :start_controllers loose-controllers)
	  (send req :stop_controllers normal-controllers)
	  (format t "start ~A manequine mode! ~%" arm)
	  )
      (progn
	(send req :start_controllers normal-controllers)
	(send req :stop_controllers loose-controllers)
	(format t "stop ~A manequine mode! ~%" arm)
	)
      )
    (setq res (ros::service-call "/pr2_controller_manager/switch_controller" req))
    (format t "controller service response: ~A ~%" (send res :ok))
    ))

(defun manequine-end ()
  (manequine-mode :arm (list "rarm" "larm") :start nil) ;;マネキンoff
  )

(defun manequine-both ()
  (manequine-mode :arm (list "rarm" "larm") :start t) ;;マネキンofn
  )