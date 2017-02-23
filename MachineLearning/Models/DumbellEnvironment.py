import Tkinter as tk
import numpy as np

class Dumbell(object):

    def __init__(self, length=2.5, mass=1.0, target=0.785398):
        self.length_0 = length # initial length (equilibrium)
		# defining system parameters
		# combined mass of the two dumbbell masses
        self.m = mass
		# arbitrary time
        t = 0.0
		# velocity, position and action
        self.omega = 0
        self.theta = 0
        self.action = 0
        # currently set up for a target of 45 deg
        self.target = target

        self.display = Display(length)

    # differential equations to be solved
    def theta_deriv(self, length, length_deriv, theta, omega):
        return -1 * omega

    def omega_deriv(self, length, length_deriv, theta, omega, spin_accel, radius, w0):
        return spin_accel*radius*radius/(length*length + radius*radius) + w0*w0*np.sin(theta)

    # function to calculate x position of mass
    def x_position(self):
        return self.length_0 * np.sin(self.theta)

    # function to calculate y position of mass
    def y_position(self, theta=None):
        if theta is None:
            theta = self.theta
        return self.length_0 * (1 - np.cos(theta))

    def calc_potential_energy(self, theta):
        return self.m * 9.81 * self.y_position(theta)

    def calc_kinetic_energy(self, omega):
	    return (self.m * self.length_0*self.length_0 * omega*omega)/2.0
        #return (self.m * self.length_0 *self.length_0 *omega*omega)/2.0

    def calc_reward(self, targetTheta):
        targetEnergy = self.calc_potential_energy(targetTheta)
        currentKineticEnergy = self.calc_kinetic_energy(self.omega) * 0.259965599
        currentPotentialEnergy = self.calc_potential_energy(self.theta)
        currentEnergy = currentKineticEnergy + currentPotentialEnergy

        #print self.y_position(self.theta), "\t", self.omega, "\t", currentKineticEnergy, "\t", currentPotentialEnergy, "\t", currentEnergy
        reward = -1 * (targetEnergy - currentEnergy) * (targetEnergy - currentEnergy)
        return reward

    def reset(self, theta=None):
        if theta is None:
            self.theta = self.target * (2 * np.random.random() - 1)
        else:
            self.theta = theta
        self.omega = 0
        return np.array([self.theta, self.omega])

    def step(self, action):
        self.action = action

	    # calculate reward of previous action (target energy being at 45deg oscillations)
        reward = self.calc_reward(self.target)

        if type(action) is list or type(action) is tuple:
            action = action[0]

		# Runge Kutta parameters
        h = 0.01 # step size for Runge Kutta
        N = 2500.0 # number of intervals for Runge Kutta
        omega1 = 0.0; omega2 = 0.0; omega3 = 0.0 # omega at half interval in Runge Kutta calculation
        wk1 = 0.0; wk2 = 0.0; wk3 = 0.0; wk4 = 0.0 # for omega
        theta1 = 0.0; theta2 = 0.0; theta3 = 0.0 # theta at half interval in Runge Kutta calculation
        ok1 = 0.0; ok2 = 0.0; ok3 = 0.0; ok4 = 0.0 # for theta

		# paramenters for finding optimal frequency
        length = self.length_0 # swing length
        length_deriv = 0.0
        radius = 0.1*length

        w0 = (9.81*length)/(length*length + radius*radius)

		# note force is thought of as the angular force (torque), constant throughout whole step
        spin_accel = action/self.m

		# step part 1
        ok1 = self.theta_deriv(length, length_deriv, self.theta, self.omega)
        wk1 = self.omega_deriv(length, length_deriv, self.theta, self.omega, spin_accel, radius, w0)
        theta1 = self.theta + ok1*(h/2.0)
        omega1 = self.omega + wk1*(h/2.0)

		# step part 2
        ok2 = self.theta_deriv(length, length_deriv, theta1, omega1)
        wk2 = self.omega_deriv(length, length_deriv, theta1, omega1, spin_accel, radius, w0)
        theta2 = self.theta + ok2*(h/2.0)
        omega2 = self.omega + wk2*(h/2.0)

		# step part 3
        ok3 = self.theta_deriv(length, length_deriv, theta2, omega2)
        wk3 = self.omega_deriv(length, length_deriv, theta2, omega2, spin_accel, radius, w0)
        theta3 = self.theta + ok3*(h)
        omega3 = self.omega + wk3*(h)

		# step part 4
        ok4 = self.theta_deriv(length, length_deriv, theta3, omega3)
        wk4 = self.omega_deriv(length, length_deriv, theta3, omega3, spin_accel, radius, w0)

		# increases theta and omega
        self.theta += (ok1 + 2.0 * ok2 + 2.0 * ok3 + ok4) * (h / 6.0)
        self.omega += (wk1 + 2.0 * wk2 + 2.0 * wk3 + wk4) * (h / 6.0)

        return np.array([self.theta, self.omega]), reward, False, {}


    def render(self):
        """
        Displays the current state of the pendulum
        """
        self.display.update(self.x_position(), self.y_position(), self.action)



class Display:

    def __init__(self, length):
        #Create a window and hide it
        self.root = tk.Tk()
        self.root.withdraw()

        self.length = length
        self.width = 400
        self.height = 400

        #Create the canvas on which the pendulum will be drawn
        self.canvas = tk.Canvas(self.root, width=self.width, height=self.height)
        self.canvas.pack()

    def update(self, x_position, y_position, action):
        """
        Updates the display of the pendulum
        """
        self.root.deiconify() #Display the window
        self.canvas.delete("all") #Clear the current pendulum drawing

        #Draw pendulum
        x = x_position*self.width / (self.length*2) + self.width/2
        y = self.height - y_position*self.height / (self.length*2)
        self.canvas.create_line(self.width/2, self.height/2, x, y)

        #Draw action
        action_text = "Action: %g" % action
        self.canvas.create_text(0, 0, anchor="nw", text=action_text)

        self.root.update()


def main():

    # cheeky code for printing to file
    import sys
    orig_stdout = sys.stdout
    f = file('dumbellOut.txt', 'w')
    sys.stdout = f

    # create dumbell enviroment
    env = Dumbell()
    # spit out initial (randomish) state
    theta, omega = env.reset()

    # step a bunch o times
    for i in range(25000):
        # step takes in an action and spits out percepts ala OpenAi
        # action can be float and continuous
        # not sure which values work yet, no noticable movement <10, breaks for >1000
        torque = 0

        if theta < 0:
            torque = -100
        else:
            torque = 100


        # going to change direction of torque depending on position of
        obs, reward, done, info = env.step(torque)
        # states are angle and angular velocity in rads (floats and continuous)
        theta, omega = obs
        print theta, "\t", omega, "\t", reward, "\t", torque




    sys.stdout = orig_stdout
    f.close()

if __name__ == '__main__':
    main()
