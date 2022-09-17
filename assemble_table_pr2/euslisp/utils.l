(defun reflect ()
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector)))

(defun manequine-both ()
  (manequine-mode :arm (list "rarm" "larm")))

(defun move-to-init ()
  (send *pr2* :angle-vector *init-pose*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation))

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
