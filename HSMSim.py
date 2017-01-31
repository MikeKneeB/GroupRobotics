import numpy as np

class HorizontalSlidingMassSimulation(object):

	#declare variables
	def __init__(self):
	
		#system parameters
		self.Theta=2*np.arctan(1)/9 #initial angle = 10
		self.omega=0 #angular velocity
		self.a=0 #angular acceleration
		self.x=0 #x position
		self.y=0 #y position
		self.t=0 #time
		self.tmax=1000 #time simulation is run for
		self.length=2 #swing length
		self.c=10 #viscous damping coefficient
		self.m1=1 #mass of sliding mass
		self.m2=3 #mass of fixed mass
		self.A=0.1 #amplitude of mass oscillation
		self.oscillation_frequency = np.sqrt(9.81/self.length) #angular frequency of mass oscillation
		self.phi=0 #initial angle of mass oscillation
	

	#1st order differential equation 2 (equation of motion of the system). Split into multiple terms to make it easier to read.
	#(to machine learning group: this is the equation you will want to use)
	def omega_deriv(self, length, theta, omega, t, c, m1, m2, A, osc_freq, phi):
	
		term1 = -2*A*A*osc_freq*omega*np.sin(osc_freq*t+phi)*np.cos(osc_freq*t+phi)

		term2 = -length*A*A*osc_freq*osc_freq*np.cos(osc_freq*t+phi)

		term3 = 9.81*length*np.sin(theta)

		term4 = 9.81*A*np.cos(theta)*np.cos(osc_freq*t+phi)

		term5 = -m2*9.81*length*np.sin(theta)

		term6 = (m1*length*length+m1*A*A*np.cos(osc_freq*t+phi)*np.cos(osc_freq*t+phi)+m2*length*length)
		
		return (-m1*(term1+term2+term3+term4)+term5)/term6 - c/((m1+m2)*length*length)*omega
		
		
	#function to calculate x position of mass
	def XPosition(self, length, theta):
		return length * np.sin(theta)


	#function to calculate y position of mass
	def YPosition(self, length, theta):
		return length * np.cos(theta)

		
	def sim(self):
		#Runge Kutta parameters
		h=0 #step size for Runge Kutta
		N=10000 #number of intervals for Runge Kutta
		omega_halfstep=0 #omega at half interval in Runge Kutta calculation
		theta_halfstep=0 #theta at half interval in Runge Kutta calculation
	
		#Link output file
		outFile = open('outFile.txt', 'w')
	
		#Ouput headings
		outFile. write( "Time \tTheta \tx \ty \tomega \ta \n")
	
		#Define step size
		h = self.tmax/N

		#loop for Runge Kutta calculation
		for n in range(0, N):

			#Calculates x, y and a
			x = XPosition(self.length, self.Theta)
			y = YPosition(self.length, self.Theta)
			a = omega_deriv(self.length, self.Theta, self.omega, self.t, self.c, self.m1, self.m2, self.A, self.oscillation_frequency, self.phi)
	
			#outputs t, theta, x, y and omega to file
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

			#calculates t, theta and omega after half a step
			self.t = self.t+h/2;
			theta_halfstep = self.omega*(h/2)+self.Theta
			omega_halfstep = -self.Theta*(h/2)+self.omega

			#uses Runge Kutta method to calculate next Theta and omega value
			self.Theta = self.Theta + h*omega_halfstep
			self.omega = self.omega + h*omega_deriv(self.length, theta_halfstep, omega_halfstep, self.t, self.c, self.m1, self.m2, self.A, self.oscillation_frequency, self.phi)

			#calculates t after a whole step
			self.t = self.t+h/2

    

		#user instructions
		print "Results have been output to a file called \"horizontal_sliding_mass_results.txt\"\n"