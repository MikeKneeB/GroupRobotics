import gym

import numpy as np
import random
import InvPenNeuralNet as NN 

NN.netReset()  # reset weights of neural network
epochs = 2000
gamma = 0.9  # since it may take several moves to goal, making gamma high
epsilon = 1
env = gym.make('Pendulum-v0')
for i in range(epochs):
	#env.render()
	status = 1
	# while game still in progress
        current_state = env.reset()
        current_state = np.array(current_state)
        current_state[0] = (current_state[0]+1)/2
        current_state[1] = (current_state[1]+1)/2
        current_state[2] = (current_state[2]+8)/16
	
	for j in range(300):
		env.render()
		# We are in state S
		# Let's run our Q function on S to get Q values for all possible actions
		qval = NN.model.predict(current_state.reshape(1, 3), batch_size=1)
		if (random.random() < epsilon):  # choose random action
                        best_action = np.random.randint(0, 80)
			#action = env.action_space.sample()
                        action = -2+(best_action)/20.0
		else:  # choose best action from Q(s,a) values
                        best_action = np.argmax(qval)
			action = -2+(best_action)/20.0
		# Take action, observe new state S'
		new_state, reward, done, info = env.step([action])
                new_state = np.array(new_state)
                new_state[0] = (new_state[0]+1)/2
                new_state[1] = (new_state[1]+1)/2
                new_state[2] = (new_state[2]+8)/16
		# Get max_Q(S',a)
		newQ = NN.model.predict(new_state.reshape(1, 3), batch_size=1)
		maxQ = np.max(newQ)
		y = np.zeros((1, 81))
		y[:] = qval[:]
		update = (reward + (gamma * maxQ))
		y[0][best_action] = update  # target output
		print("Game #: %s" % (i,))
		NN.model.fit(current_state.reshape(1, 3), y, batch_size=1, nb_epoch=1, verbose=1)
		current_state = new_state
		#clear_output(wait=True)
	if epsilon > 0.1:
		epsilon -= (1 / epochs)
