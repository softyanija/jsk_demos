(defun reflect ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun manequine-both ()
  (manequine-mode :arm (list "rarm" "larm")))

(defun rarm-move-end-pos-worldcoords (stroke-worldcoords)
  (setq stroke-rarm (transform (inverse-matrix (send (send *pr2* :rarm :end-coords) :worldrot)) stroke-worldcoords))
  (send *pr2* :rarm :move-end-pos stroke-rarm)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))

(defun larm-move-end-pos-worldcoords (stroke-worldcoords)
  (setq stroke-larm (transform (inverse-matrix (send (send *pr2* :larm :end-coords) :worldrot)) stroke-worldcoords))
  (send *pr2* :larm :move-end-pos stroke-larm)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))


(defun get-pos ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (ros::ros-info (format nil "rarm-corods is ~A" (send *pr2* :rarm :end-coords)))
  (ros::ros-info (format nil "larm-coords is ~A" (send *pr2* :larm :end-coords))))

(defun get-angle-vector ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun move-to-init ()
  (send *pr2* :angle-vector *init-pose*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :stop-grasp :larm)
  (send *ri* :stop-grasp :larm)
  (unix:sleep 1))

(defun pick-memory ()
  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :larm)
  (unix:sleep 1)

  (send *pr2* :angle-vector *pick-memory-1*)
  (send *pr2* :larm :move-end-pos #f(-100 0 0))
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *pick-memory-4*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )
