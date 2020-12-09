;; load robot and scene model
(require "package://fetcheus/fetch-utils.l")
(require "package://fetcheus/fetch-interface.l")
(require "package://jsk_maps/src/eng2-scene.l")

(unless (boundp '*scene*)
  (setq *scene* (make-eng2-7f-scene)))
(unless (boundp '*ri*)
  (fetch-init)) ;; fetch-init instantiate *fetch* and *ri*

;; show in irtviewer
(objects (list *fetch* *scene*))
;; show in "fetch Kinematics Simulator"
(send *ri* :objects (send *scene* :objects))

;; use (mapcar #'(lambda (x) (send x :name)) (send *scene* :spots)) to get list of spots
;(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :world)

;; check 20201026-soft3-model, the code to focus on the robot model
(send *irtviewer* :look-all
      (send (geo::make-bounding-box
             (flatten (send-all (send *fetch* :bodies) :vertices)))
            :grow 2.0))
;;reset-pose
(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)
(send *irtviewer* :draw-objects)

;; start to move
(setq *shelf-front* (send (send (send *scene* :spot "/eng2/7f/room73B2-table-side") :copy-worldcoords) :rotate pi/2 :z))
(send *shelf-front* :locate #f(-480 0 0))
(send *fetch* :move-to *shelf-front* :world) ;; moved in irtviewer
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :move-to *shelf-front*) ;; moved real robot
;(send *fetch* :locate #f(-200 250 0))
;; move arms
(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 680))) :debug-view t)
(send *fetch* :rarm :wrist-r :joint-angle 0)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *fetch* :rarm :move-end-pos #f(210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)

(send *irtviewer* :draw-objects) ;; update viewer

;;open shelf 
(send *ri* :start-grasp)
(send *fetch* :rarm :move-end-pos #f(-210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *fetch* :rarm :move-end-pos #f(210 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation)
(send *ri* :stop-grasp)
(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(600 250 680))) :debug-view t)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)



      

;;memos
#|
(send *fetch* :rarm :inverse-kinematics (send (send *fetch* :copy-worldcoords) :transform (make-coords :pos #f(400 0 500))) :debug-view t)
(send *fetch* :rarm :move-end-pos #f(110 0 0)
|#

;; back to original position

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
(send *ri* :wait-interpolation t)
(send *irtviewer* :draw-objects) ;; update viewer
(send *fetch* :move-to (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front") :world)
(send *irtviewer* :draw-objects) ;; update viewer
(send *ri* :move-to  (send *scene* :spot "/eng2/7f/room73B2-fetch-dock-front"))
