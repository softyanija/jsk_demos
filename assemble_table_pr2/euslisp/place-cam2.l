(defun place-cam2 ()
  (send *pr2* :angle-vector *place-cam2-2*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *ri* :start-grasp :rarm)
  (unix:sleep 1)

  (send *pr2* :angle-vector *place-cam2-3*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-4*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-5*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *pr2* :angle-vector *place-cam2-6*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)

  (send *ri* :move-gripper :rarm 0.048)
  (unix:sleep 1)

  )
  
