import gym
import numpy as np
import random

import actorcritic as ac

def get_observations(action):
    torque = np.array([action, 0])
    observations = env.step(torque)
    positions, reward, failed, info = observations

    pendulumX, pendulumY, pendulumZ = positions
    angleRad = np.arctan(pendulumY/pendulumX)
    angleDeg = int(round(np.rad2deg(angleRad)*2/18))+10
    if angleDeg == 20:
        angleDeg = 0
    velocity = int(np.round(pendulumZ)) + 8
   # print angleDeg, velocity
    return (angleDeg, velocity), reward

# resets enviroment to random position and returns the initial position and velocity
def reset_enviroment():
    observations = env.reset()
    initialPositions = observations
    pendulumX, pendulumY, pendulumZ = initialPositions
    angleRad = np.arctan(pendulumY/pendulumX)
    angleDeg = int(round(np.rad2deg(angleRad)*2/18))+10
    if angleDeg == 20:
        angleDeg = 0
    velocity = int(np.round(pendulumZ)) + 8
    return angleDeg, velocity

env = gym.make('Pendulum-v0')
env.reset()
f = open('shawac.txt', 'w')

# actions discretized
numberOfActions = 9

torqueValue = []
# works for 10 actions, too tired to generalize
for i in range(numberOfActions):
    torqueValue.append(i/2.0 - 2)

# (positions, velocities)
stateDimensions = (20, 17)

discount = 0.8
timeHorizon = 30

actor = ac.ActorCritic(numberOfActions, timeHorizon, stateDimensions, discount, 20)

f.write('epoch\treward\n')
epochs = 10000
exploration = epochs/2
for epoch in range(epochs):
    # reset enviroment and extract initial state
    state = reset_enviroment()
    cumulativeReward = 0
    print epoch
    for step in range(300):
        #print "****************************"
        #print "Epoch: ", epoch, " Step: ", step

        # get action from actor
        if epoch<exploration:
            action = random.randint(0,8)
        else:
            action = actor.getNextAction(state)
        #print "Next action: ", action
        # convert action into torque magnitude
        torque = torqueValue[action]

        # perform action on environment
        newstate, reward = get_observations(torque)
        cumulativeReward += reward

        # critique the quality of the action
        actor.critique(state, action, newstate, reward)
        #env.render()


    f.write('{}\t{}\n'.format(epoch, cumulativeReward))
    #print "Buffer: ", actor.avgBufferSize()
f.close()
