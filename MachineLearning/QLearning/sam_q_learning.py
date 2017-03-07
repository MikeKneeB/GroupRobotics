import random
import numpy
import math
from Tkinter import *
import time

class Robot:
    # initialise
    def __init__(self):
        # constants
        # mass and length of pendulum
        global mass, length
        mass = 5.18 #kg
        length = 1.82 #m

        # arrays
        # all the possible actions that could be taken and the qvalue of every state-action
        global actions, qvalues
        torque = 8.66 #Nm
        leglength = 0.1 #m
        actions = [-1*torque/leglength, 0, torque/leglength] #N

        # limits
        global numpositions, numactions, numvelocities
        numpositions = int(input('Number of possible positions: '))
        numvelocities = 201
        numactions = 3

        global maxpos, minpos, maxv, minv
        maxpos = math.pi
        minpos = -1 * maxpos
        maxv = 10.0 #rad/s
        minv = -1 * maxv #rad/s

        # the values of a single index of position and velocity
        global onestate, onevelocity
        onestate = (maxpos - minpos) / numpositions
        onevelocity = (maxv - minv) / numvelocities

        # set all qvalues to be 0
        qvalues = numpy.zeros(shape=(numpositions, numpositions, numactions))

        # UI variables
        global center, animation, canvas, pendulum, force, label, posmarker, posacmarker
        center = 200
        animation = Tk()
        canvas = Canvas(animation, width=center * 2, height=center * 2)
        canvas.pack()
        pendulum = canvas.create_line(center, center, center, center + length * 100)
        force = canvas.create_line(center, 50, center, 50, arrow=FIRST)
        label = canvas.create_text(20, 20, text=str(-1))
        posmarker = canvas.create_line(0,0,0,0)
        posacmarker = canvas.create_line(0,0,0,0)

    # returns the reward of the state
    def reward(self, initialposition, finalposition):
        # a higher reward the higher the pendulum got
        return (abs(finalposition) - abs(initialposition))*10

    # get the discrete index of the continuous state
    def stateindex(self, state):
        if (state > maxpos or state < minpos):
            raise IndexError

        state = state + maxpos

        index = int(math.floor(state/onestate))

        if (index == numpositions):
            index = numpositions - 1

        return index

    # get the discrete index of the continuous velocity
    def velocityindex(self, velocity):
        if (velocity > maxv):
            velocity = maxv
        if (velocity < minv):
            velocity = minv

        velocity = velocity + maxv

        index = int(math.floor(velocity/onevelocity))

        if (index == numvelocities):
            index = numvelocities - 1

        return index

    # return the maximum qvalue of the new state, for each possible action taken in that state
    def maxqvalue(self, state):
        sindex = self.stateindex(state)
        # the first action/velocity is temporarily the highest q-value
        max = qvalues[sindex][0][0]

        minposac = sindex
        maxposac = numpositions - sindex
        if (minposac > maxposac):
            minposac = numpositions - sindex
            maxposac = sindex
        # for each action, get the qvalue and check if it's larger than the current max
        for posacindex in range(minposac, maxposac):
            for aindex in range(1, numactions):
                q = qvalues[sindex][posacindex][aindex]
                if (q > max):
                    max = q
        # return the maximum qvalue
        return max

    # NUMERICAL MODDELLING
    # differential equations to be solved
    def theta_deriv(self, length, length_deriv, theta, omega, t):
        return -omega

    def omega_deriv(self, length, length_deriv, theta, omega, t, spin_accel, radius, w0):
        return spin_accel * radius * radius / (length * length + radius * radius) + w0 * w0 * math.sin(theta)

    # function to calculate x position of mass
    def XPosition(self, length, theta):
        return length * math.sin(theta)

    # function to calculate y position of mass
    def YPosition(self, length, theta):
        return length * math.cos(theta)

    def NextVelocity(self, InitialPosition, InitialVelocity, Force):
        # defining system parameters
        # combined mass of the two dumbbell masses
        m = mass
        # arbitrary time
        t = 0
        # velocity and position
        omega = InitialVelocity
        theta = InitialPosition

        # Runge Kutta parameters
        h = 0.01  # step size for Runge Kutta
        N = 2500  # number of intervals for Runge Kutta
        omega1 = 0
        omega2 = 0
        omega3 = 0  # omega at half interval in Runge Kutta calculation
        wk1 = 0
        wk2 = 0
        wk3 = 0
        wk4 = 0  # for omega
        theta1 = 0
        theta2 = 0
        theta3 = 0  # theta at half interval in Runge Kutta calculation
        ok1 = 0
        ok2 = 0
        ok3 = 0
        ok4 = 0  # for theta

        # paramenters for finding optimal frequency
        length_0 = length  # initial length (equilibrium)
        length_1 = length_0  # swing length
        length_deriv = 0

        spin_accel = 0
        radius = 0.1 * length_0

        w0 = (9.81 * length_0) / (length_0 * length_0 + radius * radius)

        # note force is thought of as the angular force (torque), constant throughout whole step
        spin_accel = Force / m

        # step part 1
        ok1 = self.theta_deriv(length_1, length_deriv, theta, omega, t)
        wk1 = self.omega_deriv(length_1, length_deriv, theta, omega, t, spin_accel, radius, w0)
        theta1 = theta + ok1 * (h / 2)
        omega1 = omega + wk1 * (h / 2)

        # step part 2
        ok2 = self.theta_deriv(length_1, length_deriv, theta1, omega1, t)
        wk2 = self.omega_deriv(length_1, length_deriv, theta1, omega1, t, spin_accel, radius, w0)
        theta2 = theta + ok2 * (h / 2)
        omega2 = omega + wk2 * (h / 2)

        # step part 3
        ok3 = self.theta_deriv(length_1, length_deriv, theta2, omega2, t)
        wk3 = self.omega_deriv(length_1, length_deriv, theta2, omega2, t, spin_accel, radius, w0)
        theta3 = theta + ok3 * (h)
        omega3 = omega + wk3 * (h)

        # step part 4
        ok4 = self.theta_deriv(length, length_deriv, theta3, omega3, t)
        wk4 = self.omega_deriv(length, length_deriv, theta3, omega3, t, spin_accel, radius, w0)

        # increases theta and omega
        theta = theta + (ok1 + 2 * ok2 + 2 * ok3 + ok4) * (h / 6)
        omega = omega + (wk1 + 2 * wk2 + 2 * wk3 + wk4) * (h / 6)

        return omega

    def NextPosition(self, InitialPosition, InitialVelocity, Force):
        # defining system parameters
        # combined mass of the two dumbbell masses
        m = mass
        # arbitrary time
        t = 0
        # velocity and position
        omega = InitialVelocity
        theta = InitialPosition

        # Runge Kutta parameters
        h = 0.01  # step size for Runge Kutta
        N = 2500  # number of intervals for Runge Kutta
        omega1 = 0
        omega2 = 0
        omega3 = 0  # omega at half interval in Runge Kutta calculation
        wk1 = 0
        wk2 = 0
        wk3 = 0
        wk4 = 0  # for omega
        theta1 = 0
        theta2 = 0
        theta3 = 0  # theta at half interval in Runge Kutta calculation
        ok1 = 0
        ok2 = 0
        ok3 = 0
        ok4 = 0  # for theta

        length_0 = length  # initial length (equilibrium)
        length_1 = length_0  # swing length
        length_deriv = 0

        spin_accel = Force / m  # note force is thought of as the angular force, constant force throughout whole step
        radius = 0.1 * length_0

        w0 = (9.81 * length_0) / (length_0 * length_0 + radius * radius)

        # step part 1
        ok1 = self.theta_deriv(length_1, length_deriv, theta, omega, t)
        wk1 = self.omega_deriv(length_1, length_deriv, theta, omega, t, spin_accel, radius, w0)
        theta1 = theta + ok1 * (h / 2)
        omega1 = omega + wk1 * (h / 2)

        # step part 2
        ok2 = self.theta_deriv(length_1, length_deriv, theta1, omega1, t)
        wk2 = self.omega_deriv(length_1, length_deriv, theta1, omega1, t, spin_accel, radius, w0)
        theta2 = theta + ok2 * (h / 2)
        omega2 = omega + wk2 * (h / 2)

        # step part 3
        ok3 = self.theta_deriv(length_1, length_deriv, theta2, omega2, t)
        wk3 = self.omega_deriv(length_1, length_deriv, theta2, omega2, t, spin_accel, radius, w0)
        theta3 = theta + ok3 * (h)
        omega3 = omega + wk3 * (h)

        # step part 4
        ok4 = self.theta_deriv(length_1, length_deriv, theta3, omega3, t)
        wk4 = self.omega_deriv(length_1, length_deriv, theta3, omega3, t, spin_accel, radius, w0)

        # increases theta and omega
        theta = theta + (ok1 + 2 * ok2 + 2 * ok3 + ok4) * (h / 6)
        omega = omega + (wk1 + 2 * wk2 + 2 * wk3 + wk4) * (h / 6)

        return theta

    #END OF NUMERICAL MODELLING

    # Q-Learning process with no UI and random action and position action for fast initial learning
    def qlearning_noui(self, position):
        # learning rate
        alpha = 0.5
        # discount factor
        gamma = 0.9

        # if the action causes the pendulum to make a ful rotation, the action was not good and the position should be reset
        if (position < minpos or position > maxpos):
            return maxpos + 1

        # get the corresponding index of this state
        sindex = self.stateindex(position)

        # try an action and observe the new state
        #first, get where the action will be done
        minposac = sindex
        maxposac = numpositions - sindex
        if (minposac > maxposac):
            minposac = numpositions - sindex
            maxposac = sindex
        actionposindex = int(math.floor(int(random.random()*(maxposac-minposac) + minposac)))
        # if the action is -1, do a random action
        action = int(math.floor(random.random() * numactions))

        currentposition = position
        currentvelocity = 0

        firstvelocity = self.NextVelocity(position, 0, 0)
        # if the initial position is negative, the velocity will be positive for the next half-wave
        passedmiddle = 0
        changeddirection = 0
        i = 0
        # while the current velocity is in the same direction as the initial velocity
        while (changeddirection == 0):
            i = i + 1
            thisaction = 0
            if (self.stateindex(currentposition) == actionposindex):
                thisaction = actions[action]
            else:
                thisaction = 0
            # let the pendulum move with no external forces
            nextpos = self.NextPosition(currentposition, currentvelocity, thisaction)
            nextvel = self.NextVelocity(currentposition, currentvelocity, thisaction)
            if (nextpos*position < 0):
                passedmiddle = 1
            if (passedmiddle == 1 and nextvel*firstvelocity < 0):
                changeddirection = 1
            # update the new position and velocity
            currentposition = nextpos
            currentvelocity = nextvel

            # if the action causes the pendulum to make a ful rotation, the action was not good and the position should be reset
            if (currentposition < minpos or currentposition > maxpos):
                qvalues[sindex][actionposindex][action] = -10
                return maxpos+1

        # find the new q-value
        current = qvalues[sindex][actionposindex][action]
        newq = current + alpha * (self.reward(position, currentposition) + gamma * self.maxqvalue(currentposition) - current)

        qvalues[sindex][actionposindex][action] = newq

        return currentposition

    # Q-Learning process
    def qlearning(self, position, actionposindex, action, i):
        # learning rate
        alpha = 0.5
        # discount factor
        gamma = 0.9

        # position is circular; if it goes "out of bounds", adjust it
        while (position > maxpos):
            position = position - maxpos
        while (position < minpos):
            position = position - minpos

        canvas.itemconfigure(label, text=str(i))
        canvas.coords(posmarker, center + length * math.sin(position) * 100, center + length * math.cos(position) * 100, center + length * math.sin(position) * 100, center + length * math.cos(position) * 100)
        if (action == 0):  # negative force
            canvas.coords(force, center, 50, center - 50, 50)
        else:
            if (action == 1):  # no force
                canvas.coords(force, center, 50, center, 50)
            else:
                if (action == 2):  # positive force
                    canvas.coords(force, center, 50, center + 50, 50)
        acpos = actionposindex*onestate - maxpos
        canvas.coords(posacmarker, center + length * math.sin(acpos) * 100, center + length * math.cos(acpos) * 100, center + length * math.sin(acpos) * 100, center + length * math.cos(acpos) * 100)
        # get the corresponding index of this state
        sindex = self.stateindex(position)

        # try an action and observe the new state
        #first, get where the action will be done
        minposac = sindex
        if (minposac > numpositions - sindex):
            minposac = numpositions - sindex
        if (actionposindex == -1):
            actionposindex = int(math.floor(random.random() * (numpositions - sindex*2) + minposac))
        # if the action is -1, do a random action
        if (action == -1):
            action = int(math.floor(random.random() * numactions))

        currentposition = position
        currentvelocity = 0

        firstvelocity = self.NextVelocity(position, 0, 0)
        # if the initial position is negative, the velocity will be positive for the next half-wave
        passedmiddle = 0
        changeddirection = 0
        i = 0
        # while the current velocity is in the same direction as the initial velocity
        while (changeddirection == 0):
            i = i + 1
            thisaction = 0
            if (self.stateindex(currentposition) == actionposindex):
                thisaction = actions[action]
            else:
                thisaction = 0
            # let the pendulum move with no external forces
            nextpos = self.NextPosition(currentposition, currentvelocity, thisaction)
            nextvel = self.NextVelocity(currentposition, currentvelocity, thisaction)
            if (nextpos * position < 0):
                passedmiddle = 1
            if (passedmiddle == 1 and nextvel * firstvelocity < 0):
                changeddirection = 1
            # update the new position and velocity
            currentposition = nextpos
            currentvelocity = nextvel
            # update the UI
            #canvas.coords(force, center, 50, center, 50)
            x = center + length * math.sin(currentposition) * 100
            y = center + length * math.cos(currentposition) * 100
            canvas.coords(pendulum, center, center, int(x), int(y))

            animation.update()
            time.sleep(0.01)

            # if the action causes the pendulum to make a ful rotation, the action was not good and the position should be reset
            if (currentposition < minpos or currentposition > maxpos):
                qvalues[sindex][actionposindex][action] = -10
                return maxpos+1

        # find the new q-value
        current = qvalues[sindex][actionposindex][action]
        newq = current + alpha * (self.reward(position, currentposition) + gamma * self.maxqvalue(currentposition) - current)

        qvalues[sindex][actionposindex][action] = newq

        return currentposition

    # Repeat q learning several times
    def dolearning(self):
        print('Starting')
        # do some random learning
        for num in range (0, 10000):
            position = random.random() * numpositions * onestate - maxpos
            while (position < 0.1):
                position = random.random() * numpositions * onestate - maxpos
            #print num, position
            for num2 in range (0, 10):
                position = self.qlearning_noui(position)
                if (position == maxpos+1):
                    break
#"""
        print qvalues.astype(int)

        # do some learned learning
        for num in range (0, 100):
            position = random.random() * numpositions * onestate - maxpos
            pos1 = position
            for num2 in range (0, 10):
                position = self.qlearning(position, self.nextposaction(self.stateindex(position)), self.nextaction(self.stateindex(position)), num)
                if (position == maxpos+1):
                    break
            print abs(pos1) - abs(position)

    # returns the action with the highest q-value
    def nextaction(self, pindex):
        # the first action/velocity is temporarily the highest q-value
        max = qvalues[pindex][0][0]
        index = 0
        minposac = pindex
        maxposac = numpositions - pindex
        if (minposac > maxposac):
            minposac = numpositions - pindex
            maxposac = pindex
        # for each action, get the qvalue and check if it's larger than the current max
        for posacindex in range(minposac, maxposac):
            for aindex in range(1, numactions):
                q = qvalues[pindex][posacindex][aindex]
                if (q > max):
                    max = q
                    index = aindex
        # return the maximum qvalue
        return index

    # returns the action with the highest q-value
    def nextposaction(self, pindex):
        # the first action/velocity is temporarily the highest q-value
        max = qvalues[pindex][0][0]
        pos = 0
        minposac = pindex
        maxposac = numpositions - pindex
        if (minposac > maxposac):
            minposac = numpositions - pindex
            maxposac = pindex
        # for each action, get the qvalue and check if it's larger than the current max
        for posacindex in range(minposac, maxposac):
            for aindex in range(1, numactions):
                q = qvalues[pindex][posacindex][aindex]
                if (q > max):
                    max = q
                    pos = posacindex
        # return the maximum qvalue
        return pos



dp = Robot()
dp.dolearning()
