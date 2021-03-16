#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")

(ros::roseus "announcement")

(ros::advertise "image_marker" image_view2::ImageMarker2 1)
(ros::rate 10)
(while (ros::ok)
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (case an
	  (0
	   (let* ((mrk (instance image_view2::ImageMarker2 :init))
		    (cx 320)
		       (cy 340)
		           (h 100)
			        (w 100)
				      (h/2 (/ h 2))
				             (w/2 (/ w 2)))
   (warning-message 2 "facedetect callback~%")
   (format t "cx:~3d, cy:~3d, w:~3d, h:~3d~%" cx cy w h)
   (send mrk :type image_view2::ImageMarker2::*TEXT*)
   (send mrk :position (instance geometry_msgs::Point :init
				    :x (- cx w/2) :y (- cy h/2)))
   (send mrk :text "which?")))
	  (1
	   (let* ((mrk (instance image_view2::ImageMarker2 :init))
		    (cx 320)
		       (cy 340)
		           (h 100)
			        (w 100)
				      (h/2 (/ h 2))
				             (w/2 (/ w 2)))
   (warning-message 2 "facedetect callback~%")
   (format t "cx:~3d, cy:~3d, w:~3d, h:~3d~%" cx cy w h)
   (send mrk :type image_view2::ImageMarker2::*TEXT*)
   (send mrk :position (instance geometry_msgs::Point :init
				    :x (- cx w/2) :y (- cy h/2)))
   (send mrk :text "finished?")))
	  (2
	   (let* ((mrk (instance image_view2::ImageMarker2 :init))
		    (cx 320)
		       (cy 340)
		           (h 100)
			        (w 100)
				      (h/2 (/ h 2))
				             (w/2 (/ w 2)))
   (warning-message 2 "facedetect callback~%")
   (format t "cx:~3d, cy:~3d, w:~3d, h:~3d~%" cx cy w h)
   (send mrk :type image_view2::ImageMarker2::*TEXT*)
   (send mrk :position (instance geometry_msgs::Point :init
				    :x (- cx w/2) :y (- cy h/2)))
   (send mrk :text "OK"))))
    ((ros::publish "image_marker" mrk)



