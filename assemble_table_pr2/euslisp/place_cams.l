(defun place-cam2 ()
  (send *pr2* :angle-vector *place-cam2-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:sleep 1)

  (send *pr2* :angle-vector *place-cam2-3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2500)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-4*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-5*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2500)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-6*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :move-gripper :rarm 0.050)
  (unix:sleep 1)

  (send *pr2* :angle-vector *place-cam2-5*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-4*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  )

(defun place-cam1 ()
  (send *pr2* :angle-vector *place-cam1-1*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  
  (send *pr2* :angle-vector *place-cam1-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:sleep 1)

  (send *pr2* :angle-vector *place-cam1-3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2500)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam1-4*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam1-5*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam1-6*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (rarm-move-end-pos-worldcoords #f(0 0 -12))

  (send *ri* :move-gripper :rarm 0.055)
  (unix:sleep 1)

  (send *pr2* :angle-vector *place-cam1-7*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation))
