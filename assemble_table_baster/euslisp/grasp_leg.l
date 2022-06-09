;; (send *baxter* :angle-vector (send *ri* :state :potentio-vector))
(load "package://eus_vive/euslisp/lib/baxter-interface.l")
(load "package://assemble_table_baxter/euslisp/parameters.l")
(eus_vive::baxter-init :lgripper :softhand :rgripper :softhand)
(objects (list *baxter*))

(setq *grasp_leg_init* #f(-2.41699 -30.7617 -19.9731 28.3228 63.5669 -31.6846 42.9565 35.8813 32.7173 -64.5776 5.25146 67.6758 39.9023 92.7466 8.41553))
(setq *grasp_leg_pose* #f(-2.46094 -39.7485 -5.20752 48.7573 57.019 -52.6025 49.7021 48.3838 32.7393 -64.5776 5.20752 67.5439 39.9023 92.7246 8.39355))
(setq *grasp_leg_inter1* #f(-2.43896 -48.23 -58.0957 36.2329 108.633 -101.821 40.3638 78.7939 32.6733 -64.4458 5.22949 67.6758 39.8145 92.6807 8.41553))
(setq *grasp_leg_inter2* #f(-2.48291 4.21875 -20.1709 -47.0874 69.3457 -36.2549 70.8398 106.699 32.7173 -64.3359 5.16357 67.7197 39.5728 92.6367 8.48145))
(setq *grasp_leg_inter3* #f(-2.48291 27.312 11.6675 -53.5913 107.007 -57.8979 38.4741 156.511 32.5854 -64.248 5.1416 67.6758 39.3091 92.7246 8.48145))
(setq *grasp_leg_inter4* #f(-2.46094 24.3018 18.5449 -49.1089 104.766 -41.4624 35.0684 161.763 32.6733 -64.4678 5.18555 67.6978 39.5947 92.7026 8.4375))
(setq *grasp_leg_top_pre* #f(-2.46094 24.3237 18.501 -49.1748 104.744 -41.4624 35.1123 161.785 27.8613 -54.6899 74.0479 64.8633 -29.5532 89.6924 -97.8223))
(setq *grasp_leg_top* #f(-2.43896 24.3237 18.5449 -49.1089 104.7 -41.3965 34.2334 161.807 28.0151 -39.4849 84.4189 73.3667 -45.9009 88.1104 -101.052))

(send *ri* :stop-grasp :rarm)
(send *ri* :stop-grasp :larm)
(unix:sleep 2)

(send *baxter* :angle-vector *grasp_leg_init*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_pose*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm :rotate-angle (* pi 0.0))
(unix:sleep 2)

(send *baxter* :angle-vector *grasp_leg_init*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :larm :rotate-angle (* pi -0.3))
(unix:sleep 2)

(send *baxter* :angle-vector *grasp_leg_inter1*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_inter2*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_inter3*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_inter4*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2000)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_top_pre*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 3000)
(send *ri* :wait-interpolation)

(send *baxter* :angle-vector *grasp_leg_top*)
(send *ri* :angle-vector (send *baxter* :angle-vector) 2500)
(send *ri* :wait-interpolation)

(send *ri* :start-grasp :rarm :rotate-angle (* pi -0.3))
(unix:sleep 2)
