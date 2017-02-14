//driven rotation dumbbell pendulum using RK4

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;

//define functions

//differential equations to be solved
inline double theta_deriv(double length, double length_deriv, double theta, double omega, double t)
{
    return -omega;
}

inline double omega_deriv(double length, double length_deriv , double theta, double omega, double t, double spin_accel, double radius, double w0)

{
    return spin_accel*radius*radius/(length*length + radius*radius) + w0*w0*sin(theta);
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

inline double NextVelocity(double InitialPosition, double InitialVelocity, double Force)
{
    //defining system parameters
    //combined mass of the two dumbbell masses
    double m=1;
    //arbitrary time
    double t=0;
    //velocity and position
    double omega=InitialVelocity;
    double theta=InitialPosition;

    //Runge Kutta parameters
    double h=0.01; //step size for Runge Kutta
    double N=2500; //number of intervals for Runge Kutta
    double omega1=0, omega2=0, omega3 =0; //omega at half interval in Runge Kutta calculation
    double wk1=0,wk2=0,wk3=0,wk4=0; // for omega
    double theta1 =0, theta2 =0, theta3 =0; //theta at half interval in Runge Kutta calculation
    double ok1=0,ok2=0,ok3=0,ok4=0; // for theta

    //paramenters for finding optimal frequency
    double length_0=2.5; //initial length (equilibrium)
    double length=length_0; //swing length
    double length_deriv=0;

    double spin_accel=0;
    double radius=0.1*length_0;

    double w0=(9.81*length_0)/(length_0*length_0 + radius*radius);

    //note force is thought of as the angular force (torque), constant throughout whole step
    spin_accel = Force/m;

    //step part 1
    ok1 = theta_deriv(length, length_deriv, theta, omega, t);
    wk1 = omega_deriv(length, length_deriv, theta, omega, t, spin_accel, radius, w0);
    theta1= theta + ok1*(h/2);
    omega1 = omega + wk1*(h/2);

    //step part 2
    ok2 = theta_deriv(length, length_deriv, theta1, omega1, t);
    wk2 = omega_deriv(length, length_deriv, theta1, omega1, t, spin_accel, radius, w0);
    theta2 = theta + ok2*(h/2);
    omega2 = omega + wk2*(h/2);

    //step part 3
    ok3 = theta_deriv(length, length_deriv, theta2, omega2, t);
    wk3 = omega_deriv(length, length_deriv, theta2, omega2, t, spin_accel, radius, w0);
    theta3 = theta + ok3*(h);
    omega3 = omega + wk3*(h);

    //step part 4
    ok4 = theta_deriv(length, length_deriv, theta3, omega3, t);
    wk4 = omega_deriv(length, length_deriv, theta3, omega3, t, spin_accel, radius, w0);

    //increases theta and omega
    theta = theta + (ok1+2*ok2+2*ok3+ok4)*(h/6);
    omega = omega + (wk1+2*wk2+2*wk3+wk4)*(h/6);

    return omega;
}

inline double NextPosition(double InitialPosition, double InitialVelocity, double Force)
{
    //defining system parameters
    //combined mass of the two dumbbell masses
    double m=1;
    //arbitrary time
    double t=0;
    //velocity and position
    double omega=InitialVelocity;
    double theta=InitialPosition;

    //Runge Kutta parameters
    double h=0.01; //step size for Runge Kutta
    double N=2500; //number of intervals for Runge Kutta
    double omega1=0, omega2=0, omega3 =0; //omega at half interval in Runge Kutta calculation
    double wk1=0,wk2=0,wk3=0,wk4=0; // for omega
    double theta1 =0, theta2 =0, theta3 =0; //theta at half interval in Runge Kutta calculation
    double ok1=0,ok2=0,ok3=0,ok4=0; // for theta

    double length_0=2.5; //initial length (equilibrium)
    double length=length_0; //swing length
    double length_deriv=0;

    double spin_accel=Force/m; //note force is thought of as the angular force, constant force throughout whole step
    double radius=0.1*length_0;

    double w0=(9.81*length_0)/(length_0*length_0 + radius*radius);

    //step part 1
    ok1 = theta_deriv(length, length_deriv, theta, omega, t);
    wk1 = omega_deriv(length, length_deriv, theta, omega, t, spin_accel, radius, w0);
    theta1= theta + ok1*(h/2);
    omega1 = omega + wk1*(h/2);

    //step part 2
    ok2 = theta_deriv(length, length_deriv, theta1, omega1, t);
    wk2 = omega_deriv(length, length_deriv, theta1, omega1, t, spin_accel, radius, w0);
    theta2 = theta + ok2*(h/2);
    omega2 = omega + wk2*(h/2);

    //step part 3
    ok3 = theta_deriv(length, length_deriv, theta2, omega2, t);
    wk3 = omega_deriv(length, length_deriv, theta2, omega2, t, spin_accel, radius, w0);
    theta3 = theta + ok3*(h);
    omega3 = omega + wk3*(h);

    //step part 4
    ok4 = theta_deriv(length, length_deriv, theta3, omega3, t);
    wk4 = omega_deriv(length, length_deriv, theta3, omega3, t, spin_accel, radius, w0);

    //increases theta and omega
    theta = theta + (ok1+2*ok2+2*ok3+ok4)*(h/6);
    omega = omega + (wk1+2*wk2+2*wk3+wk4)*(h/6);

    return theta;
}

int main()
{
    //Link output file
    ofstream outFile("DumbbellRotationForce.txt");

    //Ouput headings
    outFile << "Position\tVelocity\tForce" << endl;

    int NSteps=250;
    double Velocity=0; //angular velocity in rad/s
    double Position=0.1; //displacement angle in rad
    double Force=0;

    outFile << Position << "\t" << Velocity << "\t" << Force << endl;
	cout << Position << "\t" << Velocity << "\t" << Force << endl;

    for(int Step=0; Step<=NSteps; Step++){

        Force = 0; //choose force for the step here
        double InitialVelocity=Velocity;
        double InitialPosition=Position;

        Velocity = NextVelocity(InitialPosition, InitialVelocity, Force);
        Position = NextPosition(InitialPosition, InitialVelocity, Force);

        outFile << Position << "\t" << Velocity << "\t" << Force << endl;
		cout << Position << "\t" << Velocity << "\t" << Force << endl;
    }

    //user instructions
    printf ("Results have been output to a file called \"DumbbellRotationForce.txt\"\n");

    return 0;
}