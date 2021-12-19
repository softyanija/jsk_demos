#疑似アルゴリズム

robot = Robot()  # robot instance
dataset = Dataset()  # dataset to save (s, delta_a)

model = NeuralNetwork()  # action_delta model

s = robot.get_status()  # get angle_vector, force, image, sound, etc.. (any info)
s_init = s
a = heuristic(s)  # initial code to execute task.  (move arm etc...)
success = robot.excecute(a)  # execute action

if not success:
    while t < 10:  # try 10s by robot itself (this is just an example)
    # or just for i in range(10): meaning try 10 times.
        action_delta = model(s)
        success = robot.execute(s)
        if success:
            break
        s = robot.get_status()
    if not success:  # failed even after robot's try
        # maybe robot.return_to(s_init)
        delta_a = ask_human()
        dataset.save(s, delta_a)  # get and save ground truth of what to do
if num_trial % 100 == 0:  # lean model some time. In this case model is trained when new 100 data was arrived.
    model.fit(dataset)  # learn(fitting) model to the dataset
num_trial += 1
