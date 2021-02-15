#!/usr/bin/env roseus

(ros::roseus-add-msgs "std_msgs")

(setq *rect* nil)

(defun cb (msg)
  (if msg
      (setq *rect* (send msg :data))
    )
)

;;main                                                                          
(ros::roseus "touch_sensor")
(ros::subscribe "/pressure" std_msgs::Float32 #'cb)

(ros::rate 10)
(while (ros::ok)
  (if *rect*
      (progn
        ;;(ros::ros-info "hello")                                               
        (ros::ros-info "pressure: ~A" *rect*))
     )
  (ros::spin-once)
  (ros::sleep)
  )
