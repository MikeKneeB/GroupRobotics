import gym

NN.netReset()  # reset weights of neural network
epochs = 1000
gamma = 0.9  # since it may take several moves to goal, making gamma high
epsilon = 1
env = gym.make('Pendulum-v0')
state = env.reset

for i in range(epochs):
	env.render()
	status = 1
	# while game still in progress
	
	while (status == 1):
		env.render()
		# We are in state S
		# Let's run our Q function on S to get Q values for all possible actions
		qval = NN.model.predict(state.reshape(1, 64), batch_size=1)
		if (random.random() < epsilon):  # choose random action
			action = env.action_space.sample()
		else:  # choose best action from Q(s,a) values
			action = -2+((np.argmax(qval))/5
		# Take action, observe new state S'
		new_state, reward, done, info = env.step(action)
		# Get max_Q(S',a)
		newQ = NN.model.predict(new_state.reshape(1, 64), batch_size=1)
		maxQ = np.max(newQ)
		y = np.zeros((1, 21))
		y[:] = qval[:]
		update = (reward + (gamma * maxQ))
		y[0][action] = update  # target output
		print("Game #: %s" % (i,))
		NN.model.fit(state.reshape(1, 64), y, batch_size=1, nb_epoch=1, verbose=1)
		state = new_state
		if done:
			print("Episode finished after {} timesteps".format(t+1))
			status = 0
		#clear_output(wait=True)
	if epsilon > 0.1:
		epsilon -= (1 / epochs)