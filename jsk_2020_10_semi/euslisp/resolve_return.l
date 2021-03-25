(send *fetch* :angle-vector #f(340.456 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.158611 29.7989))
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)

;#f(340.456 75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0 0.158611 29.7989)
(send *fetch* :rarm :angle-vector #f(75.6304 80.2141 -11.4592 98.5487 0.0 95.111 0.0))
(send *ri* :angle-vector (send *fetch* :angle-vector) 6000)
(send *ri* :wait-interpolation)

(send *fetch* :reset-pose)
(send *ri* :angle-vector (send *fetch* :angle-vector) 4000)
(send *ri* :wait-interpolation)
