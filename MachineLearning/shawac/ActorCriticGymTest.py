import gym
import numpy as np

import actorcritic as ac

env = gym.make('Pendulum-v0')
env.reset()
f = open('shawac.txt', 'w')
f.write('epoch\treward\n')

def get_observations(action):
    torque = np.array([action, 0])
    observations = env.step(torque)
    positions, reward, failed, info = observations

    pendulumX, pendulumY, pendulumZ = positions
    angleRad = np.arctan(pendulumY/pendulumX)
    angleDeg = int(np.round(np.rad2deg(angleRad)*2))
    velocity = int(np.round(pendulumZ))
    reward = int(np.round(reward))
    #print angleDeg, velocity, reward
    return (angleDeg, velocity), reward
  
# resets enviroment to random position and returns the initial position and velocity  
def reset_enviroment():
    observations = env.reset()
    initialPositions = observations
    pendulumX, pendulumY, pendulumZ = initialPositions
    angleRad = np.arctan(pendulumY/pendulumX)
    angleDeg = int(np.round(np.rad2deg(angleRad)*2))
    velocity = int(np.round(pendulumZ))
    return angleDeg, velocity
    
# actions discretized
numberOfActions = 9

torqueValue = []
# works for 10 actions, too tired to generalize 
for i in range(numberOfActions):
    torqueValue.append(i/2.0 - 2)

# (positions, velocities)
stateDimensions = (360, 16)

discount = 0.95
timeHorizon = 100

actor = ac.ActorCritic(numberOfActions, timeHorizon, stateDimensions, discount)

for epoch in range(1000):

    # reset enviroment and extract initial state
    state = reset_enviroment()
    cumulativeReward = 0
    print epoch
    for step in range(2000):
        #print "****************************"
        #print "Epoch: ", epoch, " Step: ", step
        
        # get action from actor
        action = actor.getNextAction(state)
        #print "Next action: ", action
        # convert action into torque magnitude
        torque = torqueValue[action]
        
        # perform action on enviroment 
        newstate, reward = get_observations(torque)
        cumulativeReward += reward
        
        # critique the quality of the action
        actor.critique(state, action, newstate, reward)
        #env.render()

    f.write('{}\t{}\n'.format(epoch, cumulativeReward))
f.close()