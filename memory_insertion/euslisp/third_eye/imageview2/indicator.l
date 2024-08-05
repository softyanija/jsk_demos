#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")

(ros::roseus "indicator")

(ros::advertise "image_marker" image_view2::ImageMarker2 1)

(setq i 0)
(ros::rate 2)
(while (ros::ok)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
	(case i
	  (0
	   (send mrk :type image_view2::ImageMarker2::*TEXT*)
	   (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 70))
	   (send mrk :scale 5.0)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Hello!"))
	  (t
	   (warn "unknown case!")))
	(ros::publish "image_marker" mrk)
	(ros::spin-once)
	(ros::sleep)
	))
