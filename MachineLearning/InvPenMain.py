import gym
import InvPenNeuralNet

env = gym.make('Pendulum-v0')

env.reset()

print env.action_space

for i in range(200):
        env.render()
        #o: cos theta; sin theta; theta dot.
        NN.netReset()  # reset weights of neural network
        epochs = 1000
        gamma = 0.9  # since it may take several moves to goal, making gamma high
        epsilon = 1
        for i in range(epochs):

            state = GW.initGrid()
            status = 1
            # while game still in progress
            while (status == 1):
                # We are in state S
                # Let's run our Q function on S to get Q values for all possible actions
                qval = NN.model.predict(state.reshape(1, 64), batch_size=1)
                if (random.random() < epsilon):  # choose random action
                    action = np.random.randint(0, 4)
                else:  # choose best action from Q(s,a) values
                    action = (np.argmax(qval))
                # Take action, observe new state S'
                new_state, reward, done, info = e 
                # Observe reward
                reward = GW.getReward(new_state)
                # Get max_Q(S',a)
                newQ = NN.model.predict(new_state.reshape(1, 64), batch_size=1)
                maxQ = np.max(newQ)
                y = np.zeros((1, 4))
                y[:] = qval[:]
                if reward == -1:  # non-terminal state
                    update = (reward + (gamma * maxQ))
                else:  # terminal state
                    update = reward
                y[0][action] = update  # target output
                print("Game #: %s" % (i,))
                NN.model.fit(state.reshape(1, 64), y, batch_size=1, nb_epoch=1, verbose=1)
                state = new_state
                if reward != -1:
                    status = 0
                #clear_output(wait=True)
            if epsilon > 0.1:
                epsilon -= (1 / epochs)


        o, r, d, i = env.step(env.action_space.sample())
        print o
        print r
        if d: 
            print 'Finished'
            env.reset()

