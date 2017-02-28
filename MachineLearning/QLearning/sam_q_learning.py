import random
import numpy
import math
from Tkinter import *
import time

class DumbellPendulum:
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

        global onestate, onevelocity
        onestate = (maxpos - minpos) / numpositions
        onevelocity = (maxv - minv) / numvelocities

        # set all qvalues to be 0
        qvalues = numpy.zeros(shape=(numpositions, numactions))
        # set all qvalues to be an estimate of what they should be
        # direction of force matches direction of velocity
        for i in range(0, numpositions):
            if (i > (numpositions-1)/2+1):
                qvalues[i][0] = -1
                qvalues[i][1] = 0
                qvalues[i][2] = 1
            if (i > (numpositions-1)/2+1):
                qvalues[i][0] = 1
                qvalues[i][1] = 0
                qvalues[i][2] = -1
            if (i == 0):
                qvalues[i][1] = 100

    # returns the reward of the state
    def reward(self, position, velocity):
        if ((position == maxpos or position == minpos) and velocity == 0):
            return 100
        if (position > maxpos - math.pi/4 or position < minpos + math.pi/4):
                return 10
        if (position > maxpos - math.pi/2 or position < minpos + math.pi/2):
            return 5
        return -10

    # get the discrete index of the continuous state
    def stateindex(self, state):
        while (state > maxpos):
            state = state - maxpos
        while (state < minpos):
            state = state - minpos

        state = state + maxpos

        index = state // onestate

        if (index == numpositions):
            index = numpositions - 1

        return int(index)

    # get the discrete index of the continuous velocity
    def velocityindex(self, velocity):
        if (velocity > maxv):
            velocity = maxv
        if (velocity < minv):
            velocity = minv

        velocity = velocity + maxv

        index = velocity // onevelocity

        if (index == numvelocities):
            index = numvelocities - 1

        return int(index)

    # return the maximum qvalue of the new state, for each possible action taken in that state
    def maxqvalue(self, state, velocity):
        sindex = self.stateindex(state)
        vindex = self.velocityindex(velocity)
        # the first action/velocity is temporarily the highest q-value
        max = qvalues[sindex][0]
        # for each action, get the qvalue and check if it's larger than the current max
        for aindex in range(1, numactions):
            q = qvalues[sindex][aindex]
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

    #Q-Learning process
    def qlearning(self, position, velocity, action):
        # learning rate
        alpha = 0.5
        # discount factor
        gamma = 0.9

        # position is circular; if it goes "out of bounds", adjust it
        while (position > maxpos):
            position = position - maxpos
        while (position < minpos):
            position = position - minpos

        # try an action and observe the new state
        # if -1, do a random action
        if (action == -1):
            action = int(random.random() * numactions)

        # do the action
        currentposition = self.NextPosition(position, velocity, actions[action])
        currentvelocity = self.NextVelocity(position, velocity, actions[action])

        # if the initial position is negative, the velocity will be positive for the next half-wave
        sign = 1
        # if the initial position is positive, the velocity will be negative
        if (position > 0):
            sign = -1

        # while the current velocity is in the same direction as the initial velocity
        while (currentvelocity*sign > 0):
            # let the pendulum move with no external forces
            nextpos = dp.NextPosition(currentposition, currentvelocity, 0)
            nextvel = dp.NextVelocity(currentposition, currentvelocity, 0)
            # update the new position and velocity
            currentposition = nextpos
            currentvelocity = nextvel

        #print ('no infinite loop')

        # get the corresponding indicies of this state
        sindex = self.stateindex(position)
        vindex = self.velocityindex(velocity)

        # find the new q-value
        current = qvalues[sindex][action]
        newq = current + alpha * (self.reward(position, velocity) + gamma * self.maxqvalue(currentposition, currentvelocity) - current)

        qvalues[sindex][action] = newq

    # Repeat q learning several times
    def dolearning(self):
        print('Starting')
        #make sure each state has been done once (well, 10 times)
        for i in range(0, 100):
            # work "outwards" from rest position
            for p in range ((numpositions-1)/2 + 1, numpositions):
                for a in range (0, numactions):
                    pos = p*onestate - maxpos
                    self.qlearning(pos, 0, a)
            for p in range ((numpositions-1)/2 + 1, 0, -1):
                for a in range (0, numactions):
                    pos = p*onestate - maxpos
                    self.qlearning(pos, 0, a)
        print('done each state')
        """
        # do some random learning
        for num in range (0, 1000):
            position = int(random.random() * numpositions * onestate - maxpos)
            self.qlearning(position, 0, -1)"""

        print qvalues.astype(int)

    # returns the action with the highest q-value
    def nextaction(self, pindex, vindex):
        best = qvalues[pindex][0]
        index = 0
        for i in range(1, numactions):
            if (qvalues[pindex][i] > best):
                best = qvalues[pindex][i]
                index = i
        return index

    # returns the action with the second-highest q-value after "first"
    def secondnextaction(self, pindex, vindex, first):
        if (qvalues[pindex][1] > qvalues[pindex][2-first]):
            return 1
        else:
            return 2-first



dp = DumbellPendulum()
#dp.dolearning()

center = 200

animation = Tk()
canvas = Canvas(animation, width=center*2, height=center*2)
canvas.pack()
pendulum = canvas.create_line(center, center, center, center + length * 100)
force = canvas.create_line(center, 50, center, 50, arrow=FIRST)

position = int(random.random() * numpositions * onestate - maxpos)
while (int(position) == 0):
    position = int(random.random() * numpositions * onestate - maxpos)
velocity = 0

for num1 in range (0, 5):
    for num2 in range(0, 1000):
        # Choose the action with the best qvalue for this state
        pindex = dp.stateindex(position)
        while pindex == 0:
            position = int(random.random() * numpositions * onestate - maxpos)
            pindex = dp.stateindex(position)
        vindex = dp.velocityindex(velocity)
        nextaction = dp.nextaction(int(pindex), int(vindex))

        # Show the direction of the action
        if (nextaction == 0):  # negative force
            canvas.coords(force, center, 50, center - 50, 50)
        else:
            if (nextaction == 1):  # no force
                canvas.coords(force, center, 50, center, 50)
            else:
                if (nextaction == 2):  # positive force
                    canvas.coords(force, center, 50, center + 50, 50)

        # Get the new position and velocity after doing the action
        currentposition = dp.NextPosition(position, velocity, actions[nextaction])
        currentvelocity = dp.NextVelocity(position, velocity, actions[nextaction])

        x = center + length * math.sin(currentposition) * 100
        y = center + length * math.cos(currentposition) * 100

        canvas.coords(pendulum, center, center, int(x), int(y))

        animation.update()
        time.sleep(0.01)

        sign = 1
        canvas.coords(force, center, 50, center, 50)
        # now leave the pendulum for half a cycle
        # while the original velocity and current velocity are in the same direction
        while (sign == 1):
            nextpos = dp.NextPosition(currentposition, currentvelocity, 0)
            nextvel = dp.NextVelocity(currentposition, currentvelocity, 0)
            if currentvelocity*nextvel < 0:
                sign = -1
            currentposition = nextpos
            currentvelocity = nextvel

            # Update the drawing
            x = center + length * math.sin(currentposition) * 100
            y = center + length * math.cos(currentposition) * 100

            canvas.coords(pendulum, center, center, int(x), int(y))
            animation.update()

            time.sleep(0.01)

        position = currentposition
        velocity = 0

    time.sleep(0.5)
    position = int(random.random() * numpositions * onestate - maxpos)
    velocity = 0
