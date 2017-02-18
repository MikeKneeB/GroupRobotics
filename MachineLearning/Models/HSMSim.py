import numpy as np


class HorizontalSlidingMassSimulation(object):
    # declare variables
    def __init__(self):
        # system parameters
        self.Theta = 2 * np.arctan(1) / 9  # initial angle = 10
        self.omega = 0.0  # angular velocity
        self.a = 0.0  # angular acceleration
        self.x = 0.0  # x position
        self.y = 0.0  # y position
        self.t = 0.0  # time
        self.tmax = 1000.0  # time simulation is run for
        self.length = 2.0  # swing length
        self.c = 10.0  # viscous damping coefficient
        self.m1 = 1.0  # mass of sliding mass
        self.m2 = 3.0  # mass of fixed mass
        self.A = 0.1  # amplitude of mass oscillation
        self.oscillation_frequency = np.sqrt(9.81 / self.length)  # angular frequency of mass oscillation
        self.phi = 0.0  # initial angle of mass oscillation
        self.N = 10000
        self.Nf = float(self.N)
        self.h = self.tmax / self.Nf

    # 1st order differential equation 2 (equation of motion of the system).
    # #Split into multiple terms to make it easier to read.
    # (to machine learning group: this is the equation you will want to use)
    def omega_deriv(self, length, theta, omega, t, c, m1, m2, A, osc_freq, phi):
        term1 = -2 * A * A * osc_freq * omega * np.sin(osc_freq * t + phi) * np.cos(osc_freq * t + phi)

        term2 = -length * A * A * osc_freq * osc_freq * np.cos(osc_freq * t + phi)

        term3 = 9.81 * length * np.sin(theta)

        term4 = 9.81 * A * np.cos(theta) * np.cos(osc_freq * t + phi)

        term5 = -m2 * 9.81 * length * np.sin(theta)

        term6 = (m1 * length * length + m1 * A * A * np.cos(osc_freq * t + phi) * np.cos(
            osc_freq * t + phi) + m2 * length * length)

        return (-m1 * (term1 + term2 + term3 + term4) + term5) / term6 - c / ((m1 + m2) * length * length) * omega

    # function to calculate x position of mass
    def XPosition(self, length, theta):
        return length * np.sin(theta)

    # function to calculate y position of mass
    def YPosition(self, length, theta):
        return length * np.cos(theta)

    def sim(self):
        # Link output file
        outFile = open('outFile.txt', 'w')

        # Ouput headings
        outFile.write("Time \tTheta \tx \ty \tomega \ta \n")

        # loop for Runge Kutta calculation
        for n in range(0, self.N):
            # Calculates x, y and a
            self.x = self.XPosition(self.length, self.Theta)
            self.y = self.YPosition(self.length, self.Theta)
            self.a = self.omega_deriv(self.length, self.Theta, self.omega, self.t, self.c, self.m1, self.m2, self.A,
                            self.oscillation_frequency, self.phi)

            # outputs t, theta, x, y and omega to file
            st = str(self.t)
            sTheta = str(self.Theta)
            sx = str(self.x)
            sy = str(self.y)
            sOmega = str(self.omega)
            sa = str(self.a)

            outFile.write(st)
            outFile.write("\t")
            outFile.write(sTheta)
            outFile.write("\t")
            outFile.write(sx)
            outFile.write("\t")
            outFile.write(sy)
            outFile.write("\t")
            outFile.write(sOmega)
            outFile.write("\t")
            outFile.write(sa)
            outFile.write("\n")

            # calculates t, theta and omega after half a step
            self.t = self.t + self.h / 2.0
            theta_halfstep = self.omega * (self.h / 2.0) + self.Theta
            omega_halfstep = -self.Theta * (self.h / 2.0) + self.omega

            # uses Runge Kutta method to calculate next Theta and omega value
            self.Theta = self.Theta + self.h * omega_halfstep
            self.omega = self.omega + self.h * self.omega_deriv(self.length, theta_halfstep, omega_halfstep, self.t, self.c,
                                                      self.m1, self.m2, self.A, self.oscillation_frequency, self.phi)

            # calculates t after a whole step
            self.t = self.t + self.h / 2.0

        # user instructions
        print "Results have been output to a file called \"outFile.txt\"\n"

    #Perform a single iteration of runge kutta
    def iterate(self):
        # Calculates x, y and a
        self.x = self.XPosition(self.length, self.Theta)
        self.y = self.YPosition(self.length, self.Theta)
        self.a = self.omega_deriv(self.length, self.Theta, self.omega, self.t, self.c, self.m1, self.m2, self.A,
                             self.oscillation_frequency, self.phi)

        st = str(self.t)
        sTheta = str(self.Theta)
        sx = str(self.x)
        sy = str(self.y)
        sOmega = str(self.omega)
        sa = str(self.a)

        # calculates t, theta and omega after half a step
        self.t = self.t + self.h / 2.0
        theta_halfstep = self.omega * (self.h / 2.0) + self.Theta
        omega_halfstep = -self.Theta * (self.h / 2.0) + self.omega

        # uses Runge Kutta method to calculate next Theta and omega value
        self.Theta = self.Theta + self.h * omega_halfstep
        self.omega = self.omega + self.h * self.omega_deriv(self.length, theta_halfstep, omega_halfstep, self.t+(self.h/2.0), self.c,
                                                       self.m1, self.m2, self.A, self.oscillation_frequency,
                                                       self.phi)
        # calculates t after a whole step
        self.t = self.t + self.h / 2.0

        return (st, sTheta, sx, sy, sOmega, sa)