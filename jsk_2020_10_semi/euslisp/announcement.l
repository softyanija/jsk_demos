#!/usr/bin/env roseus

#|
;;
;; imageview2 sample program
;;
;; before invoke this script, first launch following programs
;;
roslaunch roseus_tutorials usb-camera.launch
roslaunch roseus_tutorials image-view.launch
rosrun tf static_transform_publisher 0 0 1 0 0 0 /camera /target 10
|#
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "image_view2")

(ros::load-ros-manifest "std_msgs")
;;;
;;;
(ros::roseus "imageview2-client")

(ros::advertise "image_marker" image_view2::ImageMarker2 1)


(setq an 0)
(ros::rate 10)
(while (ros::ok)
  (setq an (send (one-shot-subscribe "/announce" std_msgs::UInt16) :data))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
   (case an
          (0
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 70))
           (send mrk :scale 5.0)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Hello!")
           )
          (1
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 2.5)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Which shelf?")
           )
          (2
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 1.7)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Opening the second")
           )
          (3
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 1.8)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Opening the third")
           )
          (4
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 1.7)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Opening the forth")
           )
          (5
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 2.5)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Waiting...")
	      )
          (6
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 2.5)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :text "Returing...")
	      )
          (7
           (send mrk :type image_view2::ImageMarker2::*TEXT*)
           (send mrk :position (instance geometry_msgs::Point :init :x 320 :y 40))
           (send mrk :scale 2.5)
	   (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 0.5))
	   (send mrk :filled 1)
	   (send mrk :text "Finished!!")
		    )
          (8
           (send mrk :header :stamp (ros::time-now))
           (send mrk :type image_view2::ImageMarker2::*LINE_STRIP3D*)
           (send mrk :points3D :header :frame_id "base_link")
           (send mrk :points3D :points (list (instance geometry_msgs::Point :init :x 1.5 :y 0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y -0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y -0.5 :z 1.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.5 :z 1.0)))
           (send mrk :outline_colors (list (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0)
                                           (instance std_msgs::ColorRGBA :init :r 0.0 :g 0.0 :b 1.0 :a 1.0)))
           )
          (9
           (send mrk :header :stamp (ros::time-now))
           (send mrk :type image_view2::ImageMarker2::*LINE_LIST3D*)
           (send mrk :points3D :header :frame_id "base_link")
           (send mrk :points3D :points (list (instance geometry_msgs::Point :init :x 1.5 :y 0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y -0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y -0.5 :z 1.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.5 :z 1.0)))
           )
          (10
           (send mrk :header :stamp (ros::time-now))
           (send mrk :type image_view2::ImageMarker2::*POLYGON3D*)
           (send mrk :points3D :header :frame_id "base_link")
           (send mrk :points3D :points (list (instance geometry_msgs::Point :init :x 1.5 :y 0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y -0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y -0.5 :z 2.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.5 :z 2.0)))
           )
          (11
           (send mrk :header :stamp (ros::time-now))
           (send mrk :type image_view2::ImageMarker2::*POINTS3D*)
           (send mrk :points3D :header :frame_id "base_link")
           (send mrk :points3D :points (list (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 0.8)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 1.0)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 1.2)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 1.4)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 1.6)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 1.8)
                                             (instance geometry_msgs::Point :init :x 1.5 :y 0.0 :z 2.0)))
           (send mrk :scale 10.0)
           )
          (12
           (send mrk :action image_view2::ImageMarker2::*REMOVE*)
           (send mrk :id -1)
           )
          (13
           (send mrk :header :stamp (ros::time-now))
           (send mrk :type image_view2::ImageMarker2::*TEXT3D*)
           (send mrk :position3D :header :frame_id "base_link")
           (send mrk :position3D :point (instance geometry_msgs::Point :init :x 1.0 :y 0.0 :z 1.5))
           (send mrk :scale 1.0)
           (send mrk :text "Hello World!")
           )
          (14
           (send mrk :header :stamp (ros::time-now))
           (send mrk :type image_view2::ImageMarker2::*CIRCLE3D*)
           (send mrk :pose :header :frame_id "base_link")
           (send mrk :pose :pose :position (instance geometry_msgs::Point :init :x 1.0 :y 0.0 :z 1.5))
           (send mrk :pose :pose :orientation (instance geometry_msgs::Quaternion :init :x 0.0 :y 0.0 :z 0.0 :w 1.0))
           (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 1.0 :g 0.0 :b 0.0 :a 1.0))
           (send mrk :fill_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 0.0 :b 1.0 :a 1.0))
           (send mrk :filled 1)
           (send mrk :arc 1)
           (send mrk :angle 30.0)

           (send mrk :scale 0.5)
           )
          (t
           (warn "unknwon case")))
    (ros::publish "image_marker" mrk)
    (ros::ros-info "type ~A id ~A" (send mrk :type) (send mrk :id))
    (ros::spin-once)
    (ros::sleep)
    ))
