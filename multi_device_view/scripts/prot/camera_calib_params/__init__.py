import skrobot

rarm_init_coords = skrobot.coordinates.Coordinates([0.5, -0.3, 0.75], [0, 1.57, 0])
rarm_calib_poses_relative = [skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, 0, 0.3]),
                             skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, 0, -0.3]),
                             skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, 0.3, 0]),
                             skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0, -0.3, 0]),
                             skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [0.3, 0, 0]),
                             skrobot.coordinates.Coordinates([0.0, 0.0, 0.0], [-0.3, 0, 0])]

