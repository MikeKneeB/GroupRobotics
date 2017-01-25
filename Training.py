from IPython.display import clear_output
import random
import GridWorld as GW
import NeuralNet as NN
import numpy as np

def trainNet(init=0):
    #Training for same setup
    if init==0:
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
                new_state = GW.makeMove(state, action)
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
                clear_output(wait=True)
            if epsilon > 0.1:
                epsilon -= (1 / epochs)

    #Training for random initial player position
    #this is slow as it does 40 batch re-checks. of previous states.
    if init==1:
        NN.netReset()  # reset weights of neural network
        epochs = 3000
        gamma = 0.975
        epsilon = 1
        batchSize = 0
        buffer = 80
        replay = []
        # stores tuples of (S, A, R, S')
        h = 0
        for i in range(epochs):

            state = GW.initGridPlayer()  # using the harder state initialization function
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
                new_state = GW.makeMove(state, action)
                # Observe reward
                reward = GW.getReward(new_state)

                # Experience replay storage
                if (len(replay) < buffer):  # if buffer not filled, add to it
                    replay.append((state, action, reward, new_state))
                else:  # if buffer full, overwrite old values
                    if (h < (buffer - 1)):
                        h += 1
                    else:
                        h = 0
                    replay[h] = (state, action, reward, new_state)
                    # randomly sample our experience replay memory
                    X_train = []
                    y_train = []
                    newQ = NN.model.predict(new_state.reshape(1, 64), batch_size=1)
                    maxQ = np.max(newQ)
                    y = np.zeros((1, 4))
                    y[:] = qval[:]
                    if reward == -1:  # non-terminal state
                        update = (reward + (gamma * maxQ))
                    else:  # terminal state
                        update = reward
                    y[0][action] = update  # target output
                    X_train.append(state.reshape(64, ))
                    y_train.append(y.reshape(4, ))

                    minibatch = random.sample(replay, batchSize)
                    for memory in minibatch:
                        # Get max_Q(S',a)
                        old_state, action, reward, new_state = memory
                        old_qval = NN.model.predict(old_state.reshape(1, 64), batch_size=1)
                        newQ = NN.model.predict(new_state.reshape(1, 64), batch_size=1)
                        maxQ = np.max(newQ)
                        y = np.zeros((1, 4))
                        y[:] = old_qval[:]
                        if reward == -1:  # non-terminal state
                            update = (reward + (gamma * maxQ))
                        else:  # terminal state
                            update = reward
                        y[0][action] = update
                        X_train.append(old_state.reshape(64, ))
                        y_train.append(y.reshape(4, ))

                    X_train = np.array(X_train)
                    y_train = np.array(y_train)
                    print("Game #: %s" % (i,))
                    NN.model.fit(X_train, y_train, batch_size=batchSize+1, nb_epoch=1, verbose=1)
                    state = new_state
                if reward != -1:  # if reached terminal state, update game status
                    status = 0
                clear_output(wait=True)
            if epsilon > 0.1:  # decrement epsilon over time
                epsilon -= (1 / epochs)

    #Write for full random setup if want to.
    if init==2:
        # here
        a=1



def testAlgo(init=0):
    i = 0
    if init==0:
        state = GW.initGrid()
    elif init==1:
        state = GW.initGridPlayer()
    elif init==2:
        state = GW.initGridRand()

    print("Initial State:")
    print(GW.dispGrid(state))
    status = 1
    #while game still in progress
    while(status == 1):
        qval = NN.model.predict(state.reshape(1,64), batch_size=1)
        action = (np.argmax(qval)) #take action with highest Q-value
        print('Move #: %s; Taking action: %s' % (i, action))
        state = GW.makeMove(state, action)
        print(GW.dispGrid(state))
        reward = GW.getReward(state)
        if reward != -1:
            status = 0
            print("Reward: %s" % (reward,))
        i += 1 #If we're taking more than 10 actions, just stop, we probably can't win this game
        if (i > 10):
            print("Game lost; too many moves.")
            break