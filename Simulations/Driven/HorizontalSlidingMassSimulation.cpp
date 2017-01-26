// Name: Max Hadfield
// Date: 26.01.16
// Damped Horizontal Sliding Mass simulation using 4th order Runge Kutta method

//The swing and robot are modelled as a simple pendulum with one fixed mass (m2) at the end and one sliding mass (m1) which is able to
//oscillate perpendicularly to the pendulum with amplitude A and frequency oscillation_frequency.

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
    double theta=2*atan(1)/9; //initial angle = 10
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
    double omega1=0, omega2=0, omega3 =0; //omega at quarter intervals in Runge Kutta calculation
    double wk1=0,wk2=0,wk3=0,wk4=0; //Runge Kutta coefficients for omega
    double theta1 =0, theta2 =0, theta3 =0; //theta at half interval in Runge Kutta calculation
    double ok1=0,ok2=0,ok3=0,ok4=0; //Runge Kutta coefficients for theta

    //Link output file
    ofstream outFile("horizontal_sliding_mass_results.txt");

    //Ouput headings
    outFile << "Time \tTheta \tx \ty \tomega \ta" << endl;

    //Define step size
    h = tmax/N;

    //loop for Runge Kutta calculation
    for(int n=0; n<=N; n++){

        //Calculates x, y and a
        x = XPosition(length, theta);
        y = YPosition(length, theta);
        a = omega_deriv(length, theta, omega, t, c, m1, m2, A, oscillation_frequency, phi);

        //outputs t, theta, x, y and omega to file
        outFile << t << "\t" << theta << "\t" << x << "\t" << y << "\t" << omega << "\t" << a << endl;

        //uses 4th order Runge Kutta method to calculate next Theta and omega value

        //step part 1
        ok1 = theta_deriv(omega);
        wk1 = omega_deriv(length, theta, omega, t, c, m1, m2, A, oscillation_frequency, phi);
        theta1= theta + ok1*(h/2);
        omega1 = omega + wk1*(h/2);

        //step part 2
        ok2 = theta_deriv(omega1);
        wk2 = omega_deriv(length, theta1, omega1, t, c, m1, m2, A, oscillation_frequency, phi);
        theta2 = theta + ok2*(h/2);
        omega2 = omega + wk2*(h/2);

        //step part 3
        ok3 = theta_deriv(omega2);
        wk3 = omega_deriv(length, theta2, omega2, t, c, m1, m2, A, oscillation_frequency, phi);
        theta3 = theta + ok3*(h);
        omega3 = omega + wk3*(h);

        //step part 4
        ok4 = theta_deriv(omega3);
        wk4 = omega_deriv(length, theta3, omega3, t, c, m1, m2, A, oscillation_frequency, phi);

        //increases theta and omega
        theta = theta + (ok1+2*ok2+2*ok3+ok4)*(h/6);
        omega = omega + (wk1+2*wk2+2*wk3+wk4)*(h/6);

        //calculates t after a whole step
        t = t+h;

    }

    //user instructions
    printf ("Results have been output to a file called \"horizontal_sliding_mass_results.txt\"\n");

    return 0;
}

