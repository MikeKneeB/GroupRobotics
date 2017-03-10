"""
Author: Harry Shaw, Harry Gaskin, Chris Patmore
Date: 02/03/2017
Description: Dumbell environment adapted from numerical modelling this iteration allows only
one action in a direction at a time
"""
try:
    import Tkinter as tk
except ImportError:
    import tkinter as tk
import numpy as np


class Dummy(object):

    def __init__(self, observation_space=4):
        self.shape = (observation_space,)


class Dumbell(object):

    def __init__(self, length=2.5, mass=0.25, target=0.785398):
        self.length_0 = length  # initial length (equilibrium)
        # defining system parameters
        # combined mass of the two dumbbell masses
        self.m = mass
        # velocity and position
        self.omega = 0
        self.theta = 0
        self.action = 0
        # currently set up for a target of 45 deg
        self.target = target
        self.observation_space = Dummy()
        self.display = Display(length)
        self.cumul_action = 0

    # differential equations to be solved
    def theta_deriv(self, omega):
        return -1 * omega

    def omega_deriv(self, length, theta, spin_accel, radius, w0):
        return spin_accel*radius*radius/(length*length*2) + w0*np.sin(theta)

    # function to calculate x position of mass
    def x_position(self, theta=None):
        if theta is None:
            theta = self.theta
        return self.length_0 * np.sin(theta)

    # function to calculate y position of mass
    def y_position(self, theta=None):
        if theta is None:
            theta = self.theta
        return self.length_0 * (1 - np.cos(theta))

    def calc_potential_energy(self, theta):
        return 9.81 * self.y_position(theta)

    def calc_kinetic_energy(self, omega):
        return (self.length_0*self.length_0 * omega*omega)/2.0

    def calc_reward(self, targetTheta):
        targetEnergy = self.calc_potential_energy(targetTheta)
        currentKineticEnergy = self.calc_kinetic_energy(self.omega)
        currentPotentialEnergy = self.calc_potential_energy(self.theta)
        currentEnergy = currentKineticEnergy + currentPotentialEnergy
        reward = -0.1 * (targetEnergy - currentEnergy) * (targetEnergy - currentEnergy)

        return reward

    def reset(self):
        # position at zero, comment out for random under target
        self.theta = 0#-4/180*np.pi # self.target * (2 * np.random.random() - 1)
        # no initial speed (this could be a problem!)
        self.omega = 0  # possible random moving start 2*(2*np.random.random()-1)
        self.action = 0
        self.cumul_action = 0
        return np.array([np.cos(self.theta), np.sin(self.theta), self.omega, self.cumul_action])

    def step(self, action):
        # calculate reward of previous action (target energy being at 45deg oscillations)
        reward = self.calc_reward(self.target)

        if type(action) is list or type(action) is tuple:
            action = action[0]
        self.action = action

        if not -5 <= action <= 5:
            raise ValueError("Action must be between -5 and 5")

        # override repeat forcefull actions
        if (self.cumul_action + action) > 5 or (self.cumul_action + action) < -5:
            action = 0

        self.cumul_action += action

        # Runge Kutta parameters
        h = 0.05  # step size for Runge Kutta

        # paramenters for finding optimal frequency
        length = self.length_0  # swing length
        radius = 0.1*length

        w0 = (9.81*length)/(length*length)  # + radius*radius)

        # note force is thought of as the angular force (torque), constant throughout whole step
        spin_accel = 2*action/(self.m*radius*radius)

        # step part 1
        ok1 = self.theta_deriv(self.omega)
        wk1 = self.omega_deriv(length, self.theta, spin_accel, radius, w0)
        theta1 = self.theta + ok1*(h/2.0)
        omega1 = self.omega + wk1*(h/2.0)

        # step part 2
        ok2 = self.theta_deriv(omega1)
        wk2 = self.omega_deriv(length, theta1, spin_accel, radius, w0)
        theta2 = self.theta + ok2*(h/2.0)
        omega2 = self.omega + wk2*(h/2.0)

        # step part 3
        ok3 = self.theta_deriv(omega2)
        wk3 = self.omega_deriv(length, theta2, spin_accel, radius, w0)
        theta3 = self.theta + ok3*h
        omega3 = self.omega + wk3*h

        # step part 4
        ok4 = self.theta_deriv(omega3)
        wk4 = self.omega_deriv(length, theta3, spin_accel, radius, w0)

        # increases theta and omega
        self.theta += (ok1+2.0*ok2+2.0*ok3+ok4)*(h/6.0)
        self.omega += (wk1+2.0*wk2+2.0*wk3+wk4)*(h/6.0)

        return np.array([np.cos(self.theta), np.sin(self.theta), self.omega, self.cumul_action]), reward, False, {}

    def render(self):
        """
        Displays the current state of the pendulum
        """
        self.display.update(self.x_position(), self.y_position(),
                            self.x_position(self.target), self.y_position(self.target),
                            self.action)


class Display:

    def __init__(self, length):
        # Create a window and hide it
        self.root = tk.Tk()
        self.root.withdraw()

        self.length = length
        self.width = 400
        self.height = 400

        # Create the canvas on which the pendulum will be drawn
        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height)
        self.canvas.pack()

    def update(self, x_position, y_position, target_x, target_y, action):
        # Updates the display of the pendulum

        self.root.deiconify()  # Display the window
        self.canvas.delete("all")  # Clear the current pendulum drawing

        # Draw pendulum
        x_p = x_position * self.width / (self.length * 2) + self.width / 2
        y_p = self.height - y_position * self.height / (self.length * 2)
        self.canvas.create_line(int(self.width / 2), int(self.height / 2), int(x_p), int(y_p))

        # Draw target position
        x_t = target_x * self.width / (self.length * 2) + self.width / 2
        x_t_negative = -target_x * self.width / (self.length * 2) + self.width / 2
        y_t = self.height - target_y * self.height / (self.length * 2)
        self.canvas.create_line(self.width / 2, self.height / 2, x_t, y_t, fill="red")
        self.canvas.create_line(self.width / 2, self.height / 2, x_t_negative, y_t, fill="red")

        # Draw action
        action_text = "Action: %g" % action
        self.canvas.create_text(0, 0, anchor="nw", text=action_text)

        self.root.update()
