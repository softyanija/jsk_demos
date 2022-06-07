(setq *screw-vertical-init* #f(-7.36084 -9.31641 30.1685 -69.2798 40.9351 -0.856934 79.1016 126.123 12.085 -45.0879 59.3481 66.5332 -31.6626 78.728 -125.002))

;; <531.203 11.879 265.05 / 0.948 1.54 0.97>
(setq *rotate-table-init* #f(-1.82373 47.3071 -6.54785 -61.084 81.0791 20.2148 72.312 96.394 20.4565 -53.7231 80.3101 98.7451 -37.002 75.4761 50.0))

(setq *rarm-rotate-table-1*
      (make-coords
       :pos (v+ #f(531 11.8 265) #f(69.35 -215.35 0))
       :rpy (v+ #f(0.948 1.54 0.72) #f(0 0 -0.628))
       ))
(setq *rarm-rotate-table-2*
      (make-coords
       :pos (v+ #f(531 11.8 265) #f(251.85 -332.15 0))
       :rpy (v+ #f(0.948 1.54 0.72) #f(0 0 -1.256))
       ))
(setq *larm-rotate-table-1*
      (make-coords
       :pos (v+ #f(581 11.8 -140) #f(267.35 299.25 0))
       :rpy (v+ #f(-0.456 1.45 -0.59) #f(0 0 0))
       ))
(setq *larm-rotate-table-2*
      (make-coords
       :pos (v+ #f(581 11.8 -140) #f(109.9 185.85 0))
       :rpy (v+ #f(0.948 1.54 0.72) #f(0 0 -0.628))
       ))


(setq *rarm-pos-1*
      (make-coords
       :pos #f(713 7 300)
       :rpy #f(0.74 1.57 -2.43)
       ))
(setq *rarm-pos-1-deep*
      (make-coords
       :pos #f(713 7 295)
       :rpy #f(0.74 1.57 -2.43)
       ))
(setq *rarm-pos-2*
      (make-coords
       :pos #f(687 -13 152.5)
       :rpy #f(1.175 0.0 0.0)
       ))
(setq *rarm-pos-3*
      (make-coords
       :pos #f(680 -12 150)
       :rpy #f(0.78 0.0 0.0)
       ))

(setq *larm-pos-1*
      (make-coords
       :pos #f(700 15 -20)
       :rpy #f(-1.57 0.0 0.0)
       ))

(setq *larm-pos-1-deep*
      (make-coords
       :pos #f(702 17 -20)
       :rpy #f(-1.57 0.0 0.0)
       ))
(setq *larm-pos-2*
      (make-coords
       :pos #f(662 -7 995)
       :rpy #f(-0.39 -0.0 0.0)
       ))

(setq *rarm-test*
      (make-coords
       :pos #f(660 -100 950)
       :rpy #f(1.529 0.0 3.13)
       ))

(setq *larm-test*
      (make-coords
       :pos #f(660 100 950)
       :rpy #f(-0.0 0.0 0.0)
       ))
