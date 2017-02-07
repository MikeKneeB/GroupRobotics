import random
import numpy
import math


class InvPend:
    # initialise
    def __init__(self):

        # constants
        # mass and length of pendulum, amount of time force is applied for, goal position
        global mass, length, forcetime, goal
        mass = 1
        length = 1
        forcetime = 0.5
        goal = 0

        # arrays
        # all the possible actions that could be taken and the qvalue of every state-action
        global actions, qvalues
        actions = [-1, -0.5, 0, 0.5, 1]

        # limits
        global numpositions, numactions, numvelocities
        numpositions = int(input('Number of possible positions: '))
        numvelocities = 11
        numactions = 5

        global maxstate, minstate, maxv, minv
        maxstate = math.pi / 2
        minstate = -1 * math.pi / 2
        maxv = 2.0
        minv = -2.0

        # set all qvalues to be 0
        qvalues = numpy.zeros(shape=(numpositions, numvelocities, numactions))


    # returns the reward of a state; -10 unless the state is the goal
    def reward(self, state):
        if (state == goal):
            return 50
        else:
            return -10

    # get the discrete index of the continuous state
    def stateindex(self, state):
        if (state > maxstate):
            state = maxstate
        if (state < minstate):
            state = minstate

        index = state // ((maxstate - minstate) / numpositions)

        return int(index)

    # get the discrete index of the continuous velocity
    def velocityindex(self, velocity):
        if (velocity > maxv):
            velocity = maxv
        if (velocity < minv):
            velocity = minv

        index = velocity // ((maxv - minv) / numvelocities)

        return int(index)

    # return the maximum qvalue of the new state, for each possible action taken in that state
    def maxqvalue(self, state):
        sindex = self.stateindex(state)
        # the first action/velocity is temporarily the highest q-value
        max = qvalues[sindex][0][0]
        # for each action, get the qvalue and check if it's larger than the current max
        for aindex in range(1, numactions):
            for vindex in range(1, numvelocities):
                q = qvalues[sindex][vindex][aindex]
                if (q > max):
                    max = q
        # return the maximum qvalue
        return max

    # return the position of the pendulum after performing an action when the pendulum was in a position and had a velocity
    def stateaction(self, action, position, velocity):
        # use basic CM to find the new position
        # needs replacing by numerical model
        force = actions[action]
        angacc = force / (mass * length)
        deltatheta = velocity * forcetime + 0.5 * angacc * forcetime * forcetime
        newstate = position + deltatheta
        # check it is within the limits; if not, force it, who cares
        if (newstate > math.pi / 2):
            newstate = math.pi / 2
        if (newstate < -1 * math.pi / 2):
            newstate = -1 * math.pi / 2
        return newstate

    # return the velocity of the pendulum after performing an action when the pendulum was in a position and had a velocity
    def velocityaction(self, action, state, velocity):
        # use basic CM to find the new position
        # needs replacing by numerical model
        force = actions[action]
        angacc = force / (mass * length)
        newvelocity = velocity + angacc * forcetime
        # check it is within the limits; if not, force it, who cares
        if (newvelocity > maxv):
            newvelocity = maxv
        if (newvelocity < -1 * maxv):
            newvelocity = -1 * maxv
        return newvelocity

    def qlearning(self):
        print('Starting')
        # learning rate
        alpha = 0.5
        # discount factor
        gamma = 0.9
        # initial state
        position = 0
        velocity = 0

        for num1 in range(0, 1500):
            for num2 in range(0, 50):
                # if it is the goal state, restart from a random state
                while (position == goal):
                    position = int(random.random() * numpositions)

                # if it is out of bounds restart from a random state
                while (position > maxstate or position < minstate):
                    position = int(random.random() * numpositions)
                while (velocity > maxv or velocity < minv):
                    velocity = int(numvelocities / 2 - random.random() * numvelocities)

                # try some random action and observe the new state
                action = int(random.random() * numactions)
                newstate = self.stateaction(action, position, velocity)
                newvelocity = self.velocityaction(action, position, velocity)

                # get the corresponding indecies of this state
                sindex = self.stateindex(position)
                vindex = self.velocityindex(velocity)

                # find the new q-value
                current = qvalues[sindex][vindex][action]
                newq = current + alpha * (self.reward(position) + gamma * self.maxqvalue(newstate) - current)

                # update Q(s,a)
                qvalues[sindex][vindex][action] = newq

                position = newstate
                velocity = newvelocity
            position = int(random.random() * numpositions)
            velocity = int(numvelocities / 2 - random.random() * numvelocities)

        print qvalues.astype(int)


InvPend().qlearning()
