// Name: Joshua Torbett
// Date: 26/01/2017
// Damped Simple Pendulum Simulation using 4th order Runge Kutta

#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>
using namespace std;




//Differential equation for theta
inline double thetaDot(double omega)
{   //Change ODE here
    return omega;
}


//Differential equation for omega
inline double omegaDot(double l, double theta, double omega, double c, double m)
{   //Change ODE here
    return -(9.81/l)*sin(theta) - c/(m*l*l)*omega;
}

//function to calculate x position of mass
inline double xPos(double l, double theta)
{
    return l * sin(theta);
}

//function to calculate y position of mass
inline double yPos(double l, double theta)
{
    return l * cos(theta);
}



int main()
{
    //declare variables

    //inital conditions
    double theta = 2 * atan(1); //initial angle = pi/2
    double omega = 0; //angular velocity
    double l = 10; //swing length, m
    double c = 0; //viscous damping coefficient
    double m = 1; //mass, kg
    double t = 0; //time, s
    double tMax = 100; //time simulation is run for, s


    //calculated values
    double a = 0; //angular acceleration
    double x = 0; //x position
    double y = 0; //y position





    //Runge Kutta parameters
    double h = 0; //step size for Runge Kutta
    double N = 5000; //number of intervals for Runge Kutta
    double omega1 = 0, omega2 = 0, omega3 = 0; //omega at intervals in Runge Kutta calculation
    double wk1 = 0,wk2 = 0,wk3 = 0,wk4 = 0; // k values for omega
     double theta1 = 0, theta2 = 0, theta3 = 0; //theta at intervals in Runge Kutta calculation
    double ok1 = 0,ok2 = 0,ok3 = 0,ok4 = 0; // k values for theta
    double oError = 0, wError = 0; //errors on theta and omega

    //Link output file
    string fileName = "DSP_results.txt"; //damped simple pendulum
    ofstream outFile(fileName);

    //File headings
    outFile << "Time \tTheta \tx \ty \tomega \ta" << endl;

    //step size
    h = tMax/N;

    //loop for Runge Kutta calculation
    for(int n = 0; n <= N; n++){

        //Calculates x, y and a
        x = xPos(l, theta);
        y = yPos(l, theta);
        a = omegaDot(l, theta, omega, c, m);

        //outputs t, theta, x, y and omega to file
        outFile << t << "\t" << theta << "\t" << x << "\t" << y << "\t" << omega << "\t" << a << endl; // N*h instead of t

        //calculates t, theta and omega after half a step

        //step part 1
        ok1 = thetaDot(omega);
        wk1 = omegaDot(l, theta, omega, c, m);
        theta1= theta + ok1 * (h/2);
        omega1 = omega + wk1 * (h/2);

        //step part 2
        ok2 = thetaDot(omega1);
        wk2 = omegaDot(l, theta1, omega1, c, m);
        theta2 = theta + ok2 * (h/2);
        omega2 = omega + wk2 * (h/2);

        //step part 3
        ok3 = thetaDot(omega2);
        wk3 = omegaDot(l, theta2, omega2, c, m);
        theta3 = theta + ok3 * (h);
        omega3 = omega + wk3 * (h);

        //step part 4
        ok4 = thetaDot(omega3);
        wk4 = omegaDot(l, theta3, omega3,c, m);

        //calculates new values of theta and omega
        theta += (ok1 + 2*ok2 + 2*ok3 + ok4) * (h/6);
        omega += (wk1 + 2*wk2 + 2*wk3 + wk4) * (h/6);

        //calculates errors on theta and omega
        wError += (8.0/15.0) * pow((wk3 -wk2), 3.0) * 1/(pow((wk4-wk1),2.0));
        oError += (8.0/15.0) * pow((ok3 -ok2), 3.0) * 1/(pow((ok4-ok1),2.0));

        //calculates t after a whole step
        t = t+h;

    }

    //user instructions
    printf ("Results have been output to a file called '%s'\n",fileName.c_str());

    return 0;
}
