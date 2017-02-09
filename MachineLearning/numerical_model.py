# driven rotation dumbell pendulum using RK4

import math

# define functions
class NumMod:
    # differential equations to be solved
    def theta_deriv(self, length, length_deriv, theta, omega, t):
        return -1 * omega

    def omega_deriv(self, length, length_deriv , theta, omega, t, spin_accel, radius, w0):
        return spin_accel*radius*radius/(length*length + radius*radius) + w0*w0*math.sin(theta)


    # function to calculate x position of mass
    def XPosition(self, length, theta):
        return length * math.sin(theta)


    # function to calculate y position of mass
    def YPosition(self, length, theta):
        return length * math.cos(theta)

    def simulate(self):
        # declare variables
        # system parameters
        thetastart = 0.5*math.atan(1) # initial angle = pi/2
        theta=thetastart
        omega=0 # angular velocity
        a=0 # angular acceleration
        x=0 # x position
        y=0 # y position
        t=0 # time
        tmax=500 # time simulation is run for

        # Runge Kutta parameters
        h=0.02 # step size for Runge Kutta
        N=2500 # number of intervals for Runge Kutta

        omega1=0
        omega2=0
        omega3 =0 # omega at half interval in Runge Kutta calculation

        wk1=0
        wk2=0
        wk3=0
        wk4=0 #  for omega

        theta1 =0
        theta2 =0
        theta3 =0 # theta at half interval in Runge Kutta calculation

        ok1=0
        ok2=0
        ok3=0
        ok4=0 #  for theta

        # Link output file
        #ofstream outFileOmega("driven_rotation_optimal_freq.txt")

        # Ouput headings
        print "Time \tTheta \tx \ty \tomega \ta"
        print "w\tt\trotational_accel\tx\ty\ttheta"

        # Define step size
        N = tmax/h

        # paramenters for finding optimal frequency
        length_0=2.5 # initial length (equilibrium)
        length=length_0 # swing length
        length_deriv=0

        theta_max=0

        spin_accel=0
        radius=0.1*length_0
        max_spin=1

        w0=(9.81*length_0)/(length_0*length_0 + radius*radius)

        w = w0

        # loop for Runge Kutta calculation
        for n in range(0, int(N)):

            # calculate spin change
            # phi(spin angle) = max_spin*sin(w*t)
            spin_accel = -max_spin*w*w*math.sin(w*t)

            # Calculates x, y and a
            x = self.XPosition(length, theta)
            y = self.YPosition(length, theta)
            a = self.omega_deriv(length, length_deriv, theta, omega, t, spin_accel, radius, w0)

            print w, "\t", t, "\t", spin_accel, "\t", x, "\t", y, "\t", theta

            # calculates t, theta and omega after half a step

            # step part 1
            ok1 = self.theta_deriv(length, length_deriv, theta, omega, t)
            wk1 = self.omega_deriv(length, length_deriv, theta, omega, t, spin_accel, radius, w0)
            theta1= theta + ok1*(h/2)
            omega1 = omega + wk1*(h/2)

            # calculate spin change
            # phi(spin angle) = max_spin*sin(w*t)
            spin_accel = -max_spin*w*w*math.sin(w*(t+h/2))
            # step part 2
            ok2 = self.theta_deriv(length, length_deriv, theta1, omega1, t)
            wk2 = self.omega_deriv(length, length_deriv, theta1, omega1, t, spin_accel, radius, w0)
            theta2 = theta + ok2*(h/2)
            omega2 = omega + wk2*(h/2)

            # calculate spin change
            # phi(spin angle) = max_spin*sin(w*t)
            spin_accel = -max_spin*w*w*math.sin(w*(t+h/2))
            # step part 3
            ok3 = self.theta_deriv(length, length_deriv, theta2, omega2, t)
            wk3 = self.omega_deriv(length, length_deriv, theta2, omega2, t, spin_accel, radius, w0)
            theta3 = theta + ok3*(h)
            omega3 = omega + wk3*(h)

            # calculate spin change
            # phi(spin angle) = max_spin*sin(w*t)
            spin_accel = -max_spin*w*w*math.sin(w*(t+h))
            # step part 4
            ok4 = self.theta_deriv(length, length_deriv, theta3, omega3, t)
            wk4 = self.omega_deriv(length, length_deriv, theta3, omega3, t, spin_accel, radius, w0)

            # increases theta and omega
            theta = theta + (ok1+2*ok2+2*ok3+ok4)*(h/6)
            omega = omega + (wk1+2*wk2+2*wk3+wk4)*(h/6)

            # calculates t after a whole step
            t = t+h

        

NumMod().simulate()
