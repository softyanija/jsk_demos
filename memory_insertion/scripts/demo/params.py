import skrobot
import numpy as np

init_pose = np.array([-1.5699562 ,  3.3524606 ,  1.0337425 ,  1.570853  , -3.5117776 ,
                      -1.3622222 ,  4.714164  ,  0.9466502 ,  3.300985  ,  1.5680021 ,
                      -1.0579276 , -3.265038  ,  0.29592004,  0.        ,  0.07560766,
                      -0.3712001 ,  0.10333503, -0.6805213 , -0.22569926, -1.3155115 ,
                      -1.2769166 , -0.9133316 , -1.5833083 ,  3.2295573 ,  0.        ,
                      0.        ,  0.04608495,  0.04608495,  0.04608495,  0.04608495,
                      0.00801889,  1.043072  , -0.26748914,  1.2864872 ,  1.2099297 ,
                      -1.1000859 , -1.364676  , -3.1817577 ,  0.        ,  0.        ,
                      0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                     dtype="float32")

rarm_down = np.array([-1.5699562 ,  3.354377  ,  1.0355268 ,  1.570853  , -3.5135617 ,
                      -1.3641385 ,  4.714164  ,  0.94843435,  3.3029013 ,  1.5680021 ,
                      -1.0599099 , -3.2668881 ,  0.29584914,  0.        ,  0.07550295,
                      -0.3712001 ,  0.10333503, -0.6815991 ,  0.54859805, -1.2927411 ,
                      -1.7923342 , -1.0556413 , -0.12019031,  2.7837648 ,  0.        ,
                      0.        ,  0.04608495,  0.04608495,  0.04608495,  0.04608495,
                      0.00801889,  1.0440668 , -0.26748914,  1.2853647 ,  1.2102189 ,
                      -1.1018231 , -1.365111  , -3.1816707 ,  0.        ,  0.        ,
                      0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                     dtype="float32")

place_cam2_2 = np.array([-1.5698901 ,  3.357615  ,  1.0381038 ,  1.570853  , -3.5158746 ,
                         -1.3664513 ,  4.714164  ,  0.95081323,  3.3054783 ,  1.5681343 ,
                         -1.0622888 , -3.269201  ,  0.2958475 ,  0.        ,  0.0758171 ,
                         -0.3712001 ,  0.10333503, -0.30835348,  0.2580991 , -1.9203699 ,
                         -1.9485791 , -0.64478195, -1.5036869 ,  2.1270413 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0467198 , -0.26232886,  1.2836008 ,  1.2104503 ,
                         -1.1018231 , -1.3648065 , -3.181801  ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam2_3 = np.array([-1.5698901 ,  3.3578131 ,  1.0383021 ,  1.570853  , -3.5160067 ,
                         -1.3665833 ,  4.714164  ,  0.9509454 ,  3.3056107 ,  1.5681343 ,
                         -1.062421  , -3.2693331 ,  0.2958475 ,  0.        ,  0.07550295,
                         -0.3712001 ,  0.10333503, -0.40162343, -0.18247125, -1.5287833 ,
                         -1.8917154 , -0.8810478 , -1.8484516 ,  2.067086  ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0491241 , -0.26232886,  1.2821577 ,  1.2105081 ,
                         -1.1021127 , -1.36485   , -3.1818447 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam2_4 = np.array([-1.5698901 ,  3.3582757 ,  1.0387646 ,  1.570853  , -3.5165353 ,
                         -1.367112  ,  4.714164  ,  0.95134187,  3.305941  ,  1.5682664 ,
                         -1.0628175 , -3.2697296 ,  0.2958475 ,  0.        ,  0.07497934,
                         -0.3712001 ,  0.10333503,  0.01871312, -0.10811231, -1.587794  ,
                         -2.1406894 , -1.4187262 , -1.3706367 ,  4.6060057 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0557567 , -0.2614829 ,  1.2797524 ,  1.2108552 ,
                         -1.1032708 , -1.3655461 , -3.1824536 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam2_5 = array([-1.5698901 ,  3.3586721 ,  1.0392932 ,  1.570853  , -3.5169978 ,
                      -1.3675085 ,  4.714164  ,  0.95200264,  3.3064697 ,  1.5682664 ,
                      -1.0634782 , -3.2702582 ,  0.2958474 ,  0.        ,  0.08440413,
                      -0.3712001 ,  0.10333503,  0.14522861,  0.10024498, -1.9445834 ,
                      -2.5844922 , -1.1928838 , -1.0975312 ,  4.917834  ,  0.        ,
                      0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                      0.00801851,  1.132694  , -0.2433796 ,  1.2048666 ,  1.4090972 ,
                      -1.1191956 , -1.3759012 , -3.2064707 ,  0.        ,  0.        ,
                      0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                     dtype="float32")

place_cam2_6 = array([-1.5698901 ,  3.3589365 ,  1.0396897 ,  1.570853  , -3.5174603 ,
                      -1.3679049 ,  4.714164  ,  0.9524652 ,  3.3069322 ,  1.5683326 ,
                      -1.0638747 , -3.2707207 ,  0.2958474 ,  0.        ,  0.08503244,
                      -0.3712001 ,  0.10333503,  0.27340224,  0.2300136 , -1.8740273 ,
                      -2.437329  , -0.6980576 , -0.8522279 ,  4.632067  ,  0.        ,
                      0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                      0.00801851,  1.135264  , -0.2414339 ,  1.2048666 ,  1.4089814 ,
                      -1.1174583 , -1.3761623 , -3.2062097 ,  0.        ,  0.        ,
                      0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                     dtype="float32")

place_cam1_1 = np.array([-1.5698901 ,  3.3607867 ,  1.041606  ,  1.570853  , -3.5190463 ,
                         -1.3696231 ,  4.714164  ,  0.95385283,  3.3083858 ,  1.5683326 ,
                         -1.0655266 , -3.2722406 ,  0.2958474 ,  0.        ,  0.0842994 ,
                         -0.3712001 ,  0.10333503, -0.68259394,  0.21385597, -1.7149553 ,
                         -1.9923694 , -1.6380538 , -1.7105284 ,  2.6197362 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0668662 , -0.24041878,  1.2515298 ,  1.2217305 ,
                         -1.1086273 , -1.3677216 , -3.183585  ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam1_2 = np.array([-1.5698901 ,  3.3609188 ,  1.041606  ,  1.570853  , -3.5190463 ,
                         -1.3696231 ,  4.714164  ,  0.9540511 ,  3.3085842 ,  1.5683326 ,
                         -1.0656588 , -3.2723727 ,  0.2958474 ,  0.        ,  0.0758171 ,
                         -0.3712001 ,  0.10333503, -0.6556493 ,  0.29785872, -1.8615196 ,
                         -2.0819745 , -1.4255304 , -1.7043936 ,  2.6001139 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0515285 , -0.24041878,  1.2826387 ,  1.2108552 ,
                         -1.1018231 , -1.3649806 , -3.1820621 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam1_3 = np.array([-1.5698901 ,  3.3611171 ,  1.0416721 ,  1.570853  , -3.5191123 ,
                         -1.3696231 ,  4.714164  ,  0.9542493 ,  3.3087823 ,  1.5683326 ,
                         -1.0659231 , -3.272571  ,  0.2958474 ,  0.        ,  0.07518879,
                         -0.3712001 ,  0.10333503, -0.7445252 , -0.09060116, -1.1546756 ,
                         -2.0883377 , -1.7482243 , -1.7015655 ,  2.5969377 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0535182 , -0.24041878,  1.282318  ,  1.2109131 ,
                         -1.1018231 , -1.3649806 , -3.1820621 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam1_4 = np.array([-1.5698901 ,  3.3617117 ,  1.0422007 ,  1.570853  , -3.5197072 ,
                         -1.3702177 ,  4.714164  ,  0.9546458 ,  3.3090467 ,  1.5683326 ,
                         -1.0661875 , -3.2728353 ,  0.2958474 ,  0.        ,  0.07508407,
                         -0.3712001 ,  0.10333503, -0.18349612, -0.02749335, -1.2906564 ,
                         -1.7975404 , -1.4319003 , -1.8207364 ,  5.9678783 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0551763 , -0.24041878,  1.2816765 ,  1.2111444 ,
                         -1.1016784 , -1.3649806 , -3.1820621 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam1_5 = np.array([-1.5698901 ,  3.3617117 ,  1.0423329 ,  1.570853  , -3.5198393 ,
                         -1.3703499 ,  4.714164  ,  0.95471185,  3.3091128 ,  1.5683986 ,
                         -1.0661875 , -3.2728353 ,  0.2958474 ,  0.        ,  0.07602654,
                         -0.3712001 ,  0.10333503,  0.17839126, -0.09203928, -2.2840545 ,
                         -1.4467556 , -0.9878885 , -1.5935764 ,  5.7814426 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0555909 , -0.24041878,  1.2816765 ,  1.2111444 ,
                         -1.1018231 , -1.365024  , -3.1821058 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam1_6 = np.array([-1.5698901 ,  3.3617117 ,  1.0423989 ,  1.570853  , -3.5199053 ,
                         -1.3704821 ,  4.714164  ,  0.9549101 ,  3.3091788 ,  1.5684646 ,
                         -1.0663856 , -3.2729013 ,  0.2958474 ,  0.        ,  0.07571238,
                         -0.3712001 ,  0.10333503,  0.41044688,  0.13086835, -2.5258696 ,
                         -1.5855887 , -0.5373621 , -1.8175168 ,  5.6420836 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0601507 , -0.24033418,  1.2789506 ,  1.2118964 ,
                         -1.1028365 , -1.3655026 , -3.1824973 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

place_cam1_7 = np.array([-1.5698901 ,  3.3617117 ,  1.0425311 ,  1.570853  , -3.5199714 ,
                         -1.3704821 ,  4.714164  ,  0.9549762 ,  3.3092449 ,  1.5684646 ,
                         -1.0664518 , -3.2730336 ,  0.2958474 ,  0.        ,  0.07571238,
                         -0.3712001 ,  0.10333503,  0.41525546, -0.3245907 , -2.8181965 ,
                         -1.09331   , -0.81575614, -1.7387221 ,  5.4708757 ,  0.        ,
                         0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                         0.00801851,  1.0587413 , -0.24033418,  1.2803937 ,  1.2112602 ,
                         -1.1018231 , -1.3650675 , -3.1821492 ,  0.        ,  0.        ,
                         0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                        dtype="float32")

pick_memory_1 = np.array([-1.569824  ,  3.3636281 ,  1.0442492 ,  1.570853  , -3.5217555 ,
                          -1.3723322 ,  4.714164  ,  0.95642996,  3.310963  ,  1.5684646 ,
                          -1.0679055 , -3.2744212 ,  0.29591388,  0.        ,  0.07518879,
                          -0.3712001 ,  0.10333503, -0.5942984 , -0.24972421, -1.3572037 ,
                          -1.263901  , -1.0447835 , -1.5025992 ,  3.4326134 ,  0.        ,
                          0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                          0.00801851,  0.7182438 , -0.24033418,  1.5510727 ,  1.8394796 ,
                          -0.89277416, -1.3427474 , -3.4636085 ,  0.        ,  0.        ,
                          0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                         dtype="float32")

pick_memory_2 = np.array([-1.569824  ,  3.3644872 ,  1.0448439 ,  1.570853  , -3.5225484 ,
                          -1.3730592 ,  4.7140975 ,  0.9572229 ,  3.3116899 ,  1.5684646 ,
                          -1.0688306 , -3.2753463 ,  0.29592025,  0.        ,  0.07550295,
                          -0.3712001 ,  0.10333503, -0.5933864 , -0.24930124, -1.3564019 ,
                          -1.2639588 , -1.0447835 , -1.5025557 ,  3.43257   ,  0.        ,
                          0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                          0.00801851,  0.77370834, -0.02081035,  1.1599673 ,  1.759882  ,
                          -1.1611791 , -1.8704672 , -3.5805168 ,  0.        ,  0.        ,
                          0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                         dtype="float32")

pick_memory_3 = np.array([-1.569824  ,  3.3638263 ,  1.0445135 ,  1.570853  , -3.5222182 ,
                          -1.3727287 ,  4.7140975 ,  0.9569586 ,  3.3114254 ,  1.5684646 ,
                          -1.0684341 , -3.2749498 ,  0.29591554,  0.        ,  0.07550295,
                          -0.3712001 ,  0.10333503, -0.59305483, -0.24904746, -1.3559209 ,
                          -1.2639588 , -1.0444939 , -1.5026863 ,  3.4326134 ,  0.        ,
                          0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                          0.00801851,  0.40692946, -0.02106414,  1.1460164 ,  1.7494694 ,
                          -1.2112697 , -1.8670734 , -2.464687  ,  0.        ,  0.        ,
                          0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                         dtype="float32")

pick_memory_4 = np.array([-1.569824  ,  3.3648176 ,  1.0451082 ,  1.570853  , -3.522879  ,
                          -1.3734556 ,  4.7140317 ,  0.9574211 ,  3.3118882 ,  1.5684646 ,
                          -1.0688967 , -3.2754123 ,  0.29592136,  0.        ,  0.07497934,
                          -0.3712001 ,  0.10333503, -0.5914796 , -0.24634041, -1.3512706 ,
                          -1.2640746 , -1.0453625 , -1.5026863 ,  3.4326134 ,  0.        ,
                          0.        ,  0.04608289,  0.04608289,  0.04608289,  0.04608289,
                          0.00801851,  0.36215988, -0.02081035,  1.8345801 ,  1.5785892 ,
                          -1.1358442 , -1.3239951 , -2.4540708 ,  0.        ,  0.        ,
                          0.04316108,  0.04316108,  0.04316108,  0.04316108,  0.00748691],
                         dtype="float32")


'''
rarm_induction_init_coords = skrobot.coordinates.Coordinates()
induction_init_angle_vector = np.array([-1.5704848e+00,  4.5554123e+01,  4.2743446e+01, -1.5701544e+00,
                                        -3.7792927e+01, -3.9414707e+01, -7.8521786e+00, -1.4776426e+01,
                                        -2.9745804e+01,  1.0994394e+01, -6.0272266e+01, -4.9123852e+01,
                                        2.9614183e-01,  0.0000000e+00,  5.1312681e-02,  1.2962595e+00,
                                        1.8070649e-02, -2.4816328e-01,  1.5658520e-01, -1.6714991e+00,
                                        -1.6782597e+00, -1.2354465e+00, -1.4277110e+00, -1.1684082e+01,
                                        0.0000000e+00,  0.0000000e+00,  7.2283611e-02,  7.2283611e-02,
                                        7.2283611e-02,  7.2283611e-02,  1.2777434e-02,  4.7342056e-01,
                                        2.9929683e-01,  1.2520109e+00,  2.0709257e+00, -1.5155778e+00,
                                        -1.5172944e+00, -9.6642292e-01,  0.0000000e+00,  0.0000000e+00,
                                        2.2267133e-01,  2.2267133e-01,  2.2267133e-01,  2.2267133e-01,
                                        3.9666355e-02], dtype="float32")


init-pose = np.array([(295.978 59.7619 -9.84565 73.7113 -63.0303 69.3257 -78.1906 -182.301 -38.9906 -12.9306 -75.3743 -52.3322 -73.1604 -90.7153 185.039 4.3327 -21.2682))

(setq *rarm-down* #f(295.817 59.8189 -9.4579 73.647 -63.1298 69.339 -78.2125 -182.294 -39.0518 31.4333 -74.0693 -60.4838 -102.68
8 -6.69426 159.498 4.3327 -21.2682))

(setq *pick-memory-1* #f(295.977 41.1554 18.95 88.8618 -51.1522 
105.39 -76.9342 -198.45 -34.0499 -14.3072 -77.7627 -59.8617 -72.
4212 -86.0959 196.674 4.3267 -21.2682))

(setq *pick-memory-2* #f(295.977 44.3333 0.148724 66.4623 -66.53
07 100.829 -107.173 -205.141 -34.0024 -14.2878 -77.7168 -59.8617
 -72.4212 -86.0959 196.674 4.3087 -21.2682))

(setq *pick-memory-3* #f(295.977 23.3184 2.48495 65.6629 -69.400
6 100.229 -106.966 -141.216 -33.9786 -14.2684 -77.6892 -59.8451 
-72.4212 -86.0959 196.674 4.3207 -21.2682))

(setq *pick-memory-4* #f(295.976 20.7533 4.38979 105.115 -65.079
1 90.4484 -75.8598 -140.608 -33.8883 -14.1133 -77.432 -59.8949 -
72.4212 -86.0959 196.674 4.3147 -21.2682))

(setq *place-cam2-2* #f(295.824 59.9709 -9.36096 73.5459 -63.129
8 69.3556 -78.199 -182.304 -17.6712 14.789 -110.03 -36.9433 -111
.647 -86.1536 121.869 4.3387 -21.2682))
(setq *place-cam2-3* #f(295.824 60.1134 -9.14284 73.4632 -63.146
4 69.3556 -78.199 -182.304 -23.0104 -10.4587 -87.5935 -50.4803 -
108.389 -105.892 118.444 4.3147 -21.2682))
(setq *place-cam2-4* #f(295.828 60.4887 -8.66785 73.3254 -63.212
8 69.3754 -78.2389 -182.344 1.07312 -6.19824 -90.9746 -81.2787 -
122.654 -78.5278 263.901 4.3087 -21.2682))
(setq *place-cam2-5* #f(295.818 64.8969 -0.621938 69.0348 -64.1252 80.7306 -78.8322 -183.715 8.31717 5.73976 -111.408 -68.3472 -148.076 -62.89 281.767 4.8547 -21.2682))
(setq *place-cam2-6* #f(295.828 65.0489 -0.670407 69.0348 -64.0256 80.7372 -78.8572 -183.7 15.6657 13.1749 -107.375 -40.004 -139.65 -48.8252 265.404 4.8427 -21.2682))

 (setq *place-cam1-1* #f(295.827 61.13 -6.09412 71.7084 -63.5197 70.0019 -78.3636 -182.408 -39.1088 12.254 -98.2604 -93.8536 -114.159 -98.0072 150.101 4.8487 -21.2682))
(setq *place-cam1-2* #f(295.833 60.2512 -8.91019 73.4908 -63.1298 69.3754 -78.209 -182.319 -37.565 17.0622 -106.658 -81.6769 -119.287 -97.6557 148.977 4.3207 -21.2682))
(setq *place-cam1-3* #f(295.833 60.3652 -8.76963 73.4724 -63.1298 69.3754 -78.209 -182.319 -42.6572 -5.19492 -66.1496 -100.166 -119.655 -97.4887 148.795 4.3027 -21.2682))
(setq *place-cam1-4* #f(295.833 60.4555 -8.68239 73.4357 -63.1215 69.392 -78.209 -182.319 -10.5126 -1.57426 -73.9499 -82.0418 -102.993 -104.319 341.933 4.3387 -21.2682))
(setq *place-cam1-5* #f(295.834 60.484 -8.663 73.4265 -63.1298 69.3953 -78.209 -182.319 10.2173 -5.26763 -130.858 -56.6101 -82.9013 -91.3188 331.261 4.3327 -21.2682))
(setq *place-cam1-6* #f(295.83 60.7405 -8.37703 73.2795 -63.1879 69.4318 -78.2339 -182.344 23.5131 7.49435 -144.722 -30.7886 -90.8426 -104.132 323.269 4.3387 -21.2682))
(setq *place-cam1-7* #f(295.834 60.6597 -8.56121 73.3622 -63.1298 69.402 -78.209 -182.319 23.7886 -18.5918 -161.462 -46.7394 -62.6471 -99.6276 313.469 4.3267 -21.2682))

(defvar *x_rate_cam1*)
(defvar *y_rate_cam1*)
(defvar *memory_cam1*)
(defvar *socket_cam1*)
(defvar *memory_edge_cam2*)
(defvar *socket_cam2*)
(defvar *socket_line_cam2*)
'''
