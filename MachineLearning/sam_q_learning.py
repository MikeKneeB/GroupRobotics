import random
import numpy
import math
import Tkinter as Tk


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
        for i in range(0, numpositions):
            for j in range(0, numvelocities):
                for k in range(0, numactions):
                    pos = i * (maxstate - minstate)/numpositions
                    vel = j * (maxv - minv)/numvelocities
                    qvalues[i][j][k] = self.reward(pos, vel)

    # returns the reward of a state; -10 unless the state is the goal
    def reward(self, position, velocity):
        if (position == goal and velocity == 0):
            return 100
        if (position == goal):
            return 10
        else:
            return -1 * abs(position)

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
        m = 1
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
        length_0 = 2.5  # initial length (equilibrium)
        length = length_0  # swing length
        length_deriv = 0

        spin_accel = 0
        radius = 0.1 * length_0

        w0 = (9.81 * length_0) / (length_0 * length_0 + radius * radius)

        # note force is thought of as the angular force (torque), constant throughout whole step
        spin_accel = Force / m

        # step part 1
        ok1 = self.theta_deriv(length, length_deriv, theta, omega, t)
        wk1 = self.omega_deriv(length, length_deriv, theta, omega, t, spin_accel, radius, w0)
        theta1 = theta + ok1 * (h / 2)
        omega1 = omega + wk1 * (h / 2)

        # step part 2
        ok2 = self.theta_deriv(length, length_deriv, theta1, omega1, t)
        wk2 = self.omega_deriv(length, length_deriv, theta1, omega1, t, spin_accel, radius, w0)
        theta2 = theta + ok2 * (h / 2)
        omega2 = omega + wk2 * (h / 2)

        # step part 3
        ok3 = self.theta_deriv(length, length_deriv, theta2, omega2, t)
        wk3 = self.omega_deriv(length, length_deriv, theta2, omega2, t, spin_accel, radius, w0)
        theta3 = theta + ok3 * (h)
        omega3 = omega + wk3 * (h)

        # step part 4
        ok4 = self.theta_deriv(length, length_deriv, theta3, omega3, t)
        wk4 = self.omega_deriv(length, length_deriv, theta3, omega3, t, spin_accel, radius, w0)

        # increases theta and omega
        theta = theta + (ok1 + 2 * ok2 + 2 * ok3 + ok4) * (h / 6)
        omega = omega + (wk1 + 2 * wk2 + 2 * wk3 + wk4) * (h / 6)

        return omega

    def NextPosition(self,InitialPosition, InitialVelocity, Force):
        # defining system parameters
        # combined mass of the two dumbbell masses
        m = 1
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

        length_0 = 2.5  # initial length (equilibrium)
        length = length_0  # swing length
        length_deriv = 0

        spin_accel = Force / m  # note force is thought of as the angular force, constant force throughout whole step
        radius = 0.1 * length_0

        w0 = (9.81 * length_0) / (length_0 * length_0 + radius * radius)

        # step part 1
        ok1 = self.theta_deriv(length, length_deriv, theta, omega, t)
        wk1 = self.omega_deriv(length, length_deriv, theta, omega, t, spin_accel, radius, w0)
        theta1 = theta + ok1 * (h / 2)
        omega1 = omega + wk1 * (h / 2)

        # step part 2
        ok2 = self.theta_deriv(length, length_deriv, theta1, omega1, t)
        wk2 = self.omega_deriv(length, length_deriv, theta1, omega1, t, spin_accel, radius, w0)
        theta2 = theta + ok2 * (h / 2)
        omega2 = omega + wk2 * (h / 2)

        # step part 3
        ok3 = self.theta_deriv(length, length_deriv, theta2, omega2, t)
        wk3 = self.omega_deriv(length, length_deriv, theta2, omega2, t, spin_accel, radius, w0)
        theta3 = theta + ok3 * (h)
        omega3 = omega + wk3 * (h)

        # step part 4
        ok4 = self.theta_deriv(length, length_deriv, theta3, omega3, t)
        wk4 = self.omega_deriv(length, length_deriv, theta3, omega3, t, spin_accel, radius, w0)

        # increases theta and omega
        theta = theta + (ok1 + 2 * ok2 + 2 * ok3 + ok4) * (h / 6)
        omega = omega + (wk1 + 2 * wk2 + 2 * wk3 + wk4) * (h / 6)

        return theta

    # return the position of the pendulum after performing an action when the pendulum was in a position and had a velocity
    def stateaction(self, action, position, velocity):
        # use basic CM to find the new position
        # needs replacing by numerical model
        force = actions[action]
        angacc = force / (mass * length)
        deltatheta = velocity * forcetime + 0.5 * angacc * forcetime * forcetime
        newstate = position + deltatheta
        # check it is within the limits; if not, force it, who cares
        if (newstate > maxstate):
            newstate = maxstate
        if (newstate < minstate):
            newstate = minstate
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

        for num1 in range(0, 2000):
            for num2 in range(0, 100):
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
                newstate = self.NextPosition(action, position, velocity)
                newvelocity = self.NextVelocity(action, position, velocity)

                # get the corresponding indecies of this state
                sindex = self.stateindex(position)
                vindex = self.velocityindex(velocity)

                # find the new q-value
                current = qvalues[sindex][vindex][action]
                newq = current + alpha * (self.reward(position, velocity) + gamma * self.maxqvalue(newstate) - current)

                # update Q(s,a)
                qvalues[sindex][vindex][action] = newq

                position = newstate
                velocity = newvelocity
            position = int(random.random() * numpositions)
            velocity = int(numvelocities / 2 - random.random() * numvelocities)

        print qvalues.astype(int)

    def nextaction(self, position, velocity):
        best = qvalues[position][velocity][0]
        index = 0
        for i in range(1, numactions):
            if (qvalues[position][velocity][i] > best):
                best = qvalues[position][velocity][i]
                index = i
        return index

    def graphics(self):
        # do some learning
        self.qlearning()
        
        # set the center of the pendulum at the center of the canvas
        center = 200
        
        # make the GUI
        root = Tk.Tk()
        canvas = Tk.Canvas(root, width=400, height=400)
        pendulum = canvas.create_line(center, center, center, center+length)
        canvas.pack()
        root.mainloop()

        position = 0
        velocity = 0

        for i in range(0, 50):
            # draw the new state of the pendulum
            x = center + length * math.cos(position)
            y = center + length * math.sin(position)
            pendulum = canvas.create_line(center, center, x, y)
            canvas.pack()
            root.mainloop()

            # get the next state
            position = self.NextPosition(self.nextaction(position, velocity), position, velocity)
            velocity = self.NextVelocity(self.nextaction(position, velocity), position, velocity)

InvPend().graphics()
