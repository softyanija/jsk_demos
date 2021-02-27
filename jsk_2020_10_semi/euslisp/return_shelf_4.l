;;return_shelf_4.l

;;取っ手を掴み直す
(send *fetch* :rarm :move-end-pos #f(0 50 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)
(send *fetch* :rarm :move-end-pos #f(50 0 0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)
(send *ri* :start-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

;;引き出しを戻す
(setq pull-range (float-vector (+ dist 70) 0 0))

(send *fetch* :rarm :move-end-pos pull-range)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)

(send *ri* :stop-grasp :effort 60)
(unix:sleep 1)
(send *ri* :wait-interpolation)

(send *fetch* :rarm :move-end-pos (v- #f(0 0 0) pull-range))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000 :use-torso nil)
(send *ri* :wait-interpolation)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)
