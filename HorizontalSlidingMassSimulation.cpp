// Name: Max Hadfield
// Date: 26.01.16
// Damped Horizontal Sliding Mass simulation using 2nd order Runge Kutta method

//The swing and robot are modelled as a simple pendulum with one fixed mass (m2) at the end and one sliding mass (m1) which is able to oscillate
//perpendicularly to the pendulum with amplitude A and frequency oscillation_frequency.

#include <cmath>
#include <fstream>
#include <iomanip>

using namespace std;

//define functions

//the second order differential equation describing the system behaviour was separated into two coupled first order differential equations
//so we it could be solved using the Runge Kutta method

//1st order differential equation 1 (d theta /dt = omega)
inline double theta_deriv(double omega)
{
    return omega;
}

//1st order differential equation 2 (equation of motion of the system). Split into multiple terms to make it wasier to read.
//(to machine learning group: this is the equation you will want to use)
inline double omega_deriv(double length, double theta, double omega, double t, double c, double m1, double m2, double A, double osc_freq, double phi)
{
    double term1 = -2*A*A*osc_freq*omega*sin(osc_freq*t+phi)*cos(osc_freq*t+phi);

    double term2 = -length*A*A*osc_freq*osc_freq*cos(osc_freq*t+phi);

    double term3 = 9.81*length*sin(theta);

    double term4 = 9.81*A*cos(theta)*cos(osc_freq*t+phi);

    double term5 = -m2*9.81*length*sin(theta);

    double term6 = (m1*length*length+m1*A*A*cos(osc_freq*t+phi)*cos(osc_freq*t+phi)+m2*length*length);

    return (-m1*(term1+term2+term3+term4)+term5)/term6 - c/((m1+m2)*length*length)*omega;
}

//function to calculate x position of mass
inline double XPosition(double length, double theta)
{
    return length * sin(theta);
}

//function to calculate y position of mass
inline double YPosition(double length, double theta)
{
    return length * cos(theta);
}



int main()
{
    //declare variables

    //system parameters
    double Theta=2*atan(1)/9; //initial angle = 10
    double omega=0; //angular velocity
    double a=0; //angular acceleration
    double x=0; //x position
    double y=0; //y position
    double t=0; //time
    double tmax=1000; //time simulation is run for
    double length=2; //swing length
    double c=10; //viscous damping coefficient
    double m1=1; //mass of sliding mass
    double m2=3; //mass of fixed mass
    double A=0.1; //amplitude of mass oscillation
    double oscillation_frequency = sqrt(9.81/length); //angular frequency of mass oscillation
    double phi=0; //initial angle of mass oscillation

    //Runge Kutta parameters
    double h=0; //step size for Runge Kutta
    double N=10000; //number of intervals for Runge Kutta
    double omega_halfstep=0; //omega at half interval in Runge Kutta calculation
    double theta_halfstep=0; //theta at half interval in Runge Kutta calculation

    //Link output file
    ofstream outFile("horizontal_sliding_mass_results.txt");

    //Ouput headings
    outFile << "Time \tTheta \tx \ty \tomega \ta" << endl;

    //Define step size
    h = tmax/N;

    //loop for Runge Kutta calculation
    for(int n=0; n<=N; n++){

        //Calculates x, y and a
        x = XPosition(length, Theta);
        y = YPosition(length, Theta);
        a = omega_deriv(length, Theta, omega, t, c, m1, m2, A, oscillation_frequency, phi);

        //outputs t, theta, x, y and omega to file
        outFile << t << "\t" << Theta << "\t" << x << "\t" << y << "\t" << omega << "\t" << a << endl;

        //calculates t, theta and omega after half a step
        t = t+h/2;
        theta_halfstep = omega*(h/2)+Theta;
        omega_halfstep = -Theta*(h/2)+omega;

        //uses Runge Kutta method to calculate next Theta and omega value
        Theta = Theta + h*theta_deriv(omega_halfstep);
        omega = omega + h*omega_deriv(length, theta_halfstep, omega_halfstep, t, c, m1, m2, A, oscillation_frequency, phi);

        //calculates t after a whole step
        t = t+h/2;

    }

    //user instructions
    printf ("Results have been output to a file called \"horizontal_sliding_mass_results.txt\"\n");

    return 0;
}

