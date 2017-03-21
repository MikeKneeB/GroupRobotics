//Written by Daniel Corless

#include <cmath>
#include <iostream>
#include <fstream>

#include "PendulumSystem.h"

//Constructor
PendulumSystem::PendulumSystem() {}

//The destructor deletes all the data from the members of the array
PendulumSystem::~PendulumSystem() {
    int NPendulums = fPendulums.size();
    for (int iComponent = 0; iComponent < NPendulums; ++iComponent) {
        delete fPendulums[iComponent];
        fPendulums.erase(fPendulums.begin() + iComponent);
    }
}

//Differential equation for thetaDot
double::PendulumSystem::thetaDot(double omega)
{
    return omega;
}

//Differential equation for omegaDot
double::PendulumSystem::omegaDot(double d1, double d2, double l1,double l2, double l3,double l4, double l5, double r2,double r3, double a1 ,double a2,double t1,double a1Dot, double a2Dot, double t1Dot, double c,double m1, double m2, double m3, double m4, double m5, double m6, double m7, double t2, double t3, double t4, double t5, double t2Dot, double t3Dot, double t4Dot, double t5Dot, double t2DDot, double t3DDot, double t4DDot, double t5DDot)
{
    double M=m1+m2+m3+m4+m5;
    double g = 9.807;

    double b = (-M*(d2*a2Dot*a2Dot*sin(a1-a2)+l1*t1Dot*t1Dot*sin(a1-t1))-(m2*l2+m4*r2)*(t2DDot*cos(a1-t2)+t2Dot*t2Dot*sin(a1-t2))-(m3*l3+m5*r3)*(t3DDot*cos(a1-t3)+t3Dot*t3Dot*sin(a1-t3))-(m4*l4)*(t4DDot*cos(a1-t4)+t4Dot*t4Dot*sin(a1-t4))-(m5*l5)*(t5DDot*cos(a1-t5)+t5Dot*t5Dot*sin(a1-t5))-(M+m6+m7)*g*sin(a1)-m7*d2*a2Dot*a2Dot*sin(a1-a2)-((M+m7)*cos(a1-a2)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(
                    M*(-d1*a1Dot*a1Dot*cos(a1-t1)*sin(a2-t1)-d2*a2Dot*a2Dot*cos(a2-t1)*sin(a2-t1))-(m2*l2+m4*r2)*(t2DDot*sin(t2-t1)*sin(a2-t1)+t2Dot*t2Dot*cos(t2-t1)*sin(a2-t1))-(m3*l3+m5*r3)*(t3DDot*sin(t3-t1)*sin(a2-t1)+t3Dot*t3Dot*cos(t3-t1)*sin(a2-t1))-(m4*l4)*(t4DDot*sin(t4-t1)*sin(a2-t1)+t4Dot*t4Dot*cos(t4-t1)*sin(a2-t1))-(m5*l5)*(t5DDot*sin(t5-t1)*sin(a2-t1)+t5Dot*t5Dot*cos(t5-t1)*sin(a2-t1))
                    +M*g*cos(t1)*sin(t1-a2)+M*l1*t1Dot*t1Dot*sin(t1-a2)+m7*d1*a1Dot*a1Dot*sin(a1-a2)-m7*g*sin(a2))-((M+m7)*cos(a1-t1)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(M*d1*a1Dot*a1Dot*cos(a1-a2)*sin(a2-t1)+(M*M/(M+m7))*(l1*t1Dot*t1Dot*cos(a2-t1)*sin(a2-t1))-(M*sin(t1-a2))/(M+m7)*((m2*l2+m4*r2)*(t2DDot*sin(t2-a2)+t2Dot*t2Dot*cos(t2-a2))+(m3*l3+m5*r3)*(t3DDot*sin(t3-a2)+t3Dot*t3Dot*cos(t3-a2))+(m4*l4)*(t4DDot*sin(t4-a2)+t4Dot*t4Dot*cos(t4-a2))+(m5*l5)*(t5DDot*sin(t5-a2)+t5Dot*t5Dot*cos(t5-a2)))
                    +M*d2*a2Dot*a2Dot*sin(a2-t1)-m7/(M+m7)*(m2*l2+m4*r2)*(t2DDot*cos(t1-t2)+t2Dot*t2Dot*sin(t1-t2))-m7/(M+m7)*(m3*l3+m5*r3)*(t3DDot*cos(t1-t3)+t3Dot*t3Dot*sin(t1-t3))-m7/(M+m7)*(m4*l4)*(t4DDot*cos(t1-t4)+t4Dot*t4Dot*sin(t1-t4))-m7/(M+m7)*(m5*l5)*(t5DDot*cos(t1-t5)+t5Dot*t5Dot*sin(t1-t5))-M*g*cos(a2)*sin(t1-a2)))/((M+m6+m7+((M+m7)*cos(a1-a2)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(-(M+m7)*cos(a1-a2)+M*cos(a2-t1)*cos(a1-t1))+((M+m7)*cos(a1-t1)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(-M*cos(a1-t1)+M*cos(a2-t1)*cos(a1-a2)))*d1) -2*c*a1Dot;
    return b;


}

//Differential equation for thetaDot2
double::PendulumSystem::thetaDot2(double omega2)
{
    return omega2;
}

//Differential equation for omegaDot2
double::PendulumSystem::omegaDot2(double d1, double d2, double l1,double l2, double l3,double l4, double l5, double r2,double r3, double a1 ,double a2,double t1,double a1Dot, double a2Dot, double t1Dot, double c,double m1, double m2, double m3, double m4, double m5, double m6, double m7, double t2, double t3, double t4, double t5, double t2Dot, double t3Dot, double t4Dot, double t5Dot, double t2DDot, double t3DDot, double t4DDot, double t5DDot)
{
    double M=m1+m2+m3+m4+m5;
    double g = 9.807;

    double a = (-M*(d2*a2Dot*a2Dot*sin(a1-a2)+l1*t1Dot*t1Dot*sin(a1-t1))-(m2*l2+m4*r2)*(t2DDot*cos(a1-t2)+t2Dot*t2Dot*sin(a1-t2))-(m3*l3+m5*r3)*(t3DDot*cos(a1-t3)+t3Dot*t3Dot*sin(a1-t3))-(m4*l4)*(t4DDot*cos(a1-t4)+t4Dot*t4Dot*sin(a1-t4))-(m5*l5)*(t5DDot*cos(a1-t5)+t5Dot*t5Dot*sin(a1-t5))-(M+m6+m7)*g*sin(a1)-m7*d2*a2Dot*a2Dot*sin(a1-a2)-((M+m7)*cos(a1-a2)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(
                    M*(-d1*a1Dot*a1Dot*cos(a1-t1)*sin(a2-t1)-d2*a2Dot*a2Dot*cos(a2-t1)*sin(a2-t1))-(m2*l2+m4*r2)*(t2DDot*sin(t2-t1)*sin(a2-t1)+t2Dot*t2Dot*cos(t2-t1)*sin(a2-t1))-(m3*l3+m5*r3)*(t3DDot*sin(t3-t1)*sin(a2-t1)+t3Dot*t3Dot*cos(t3-t1)*sin(a2-t1))-(m4*l4)*(t4DDot*sin(t4-t1)*sin(a2-t1)+t4Dot*t4Dot*cos(t4-t1)*sin(a2-t1))-(m5*l5)*(t5DDot*sin(t5-t1)*sin(a2-t1)+t5Dot*t5Dot*cos(t5-t1)*sin(a2-t1))
                    +M*g*cos(t1)*sin(t1-a2)+M*l1*t1Dot*t1Dot*sin(t1-a2)+m7*d1*a1Dot*a1Dot*sin(a1-a2)-m7*g*sin(a2))-((M+m7)*cos(a1-t1)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(M*d1*a1Dot*a1Dot*cos(a1-a2)*sin(a2-t1)+(M*M/(M+m7))*(l1*t1Dot*t1Dot*cos(a2-t1)*sin(a2-t1))-(M*sin(t1-a2))/(M+m7)*((m2*l2+m4*r2)*(t2DDot*sin(t2-a2)+t2Dot*t2Dot*cos(t2-a2))+(m3*l3+m5*r3)*(t3DDot*sin(t3-a2)+t3Dot*t3Dot*cos(t3-a2))+(m4*l4)*(t4DDot*sin(t4-a2)+t4Dot*t4Dot*cos(t4-a2))+(m5*l5)*(t5DDot*sin(t5-a2)+t5Dot*t5Dot*cos(t5-a2)))
                    +M*d2*a2Dot*a2Dot*sin(a2-t1)-m7/(M+m7)*(m2*l2+m4*r2)*(t2DDot*cos(t1-t2)+t2Dot*t2Dot*sin(t1-t2))-m7/(M+m7)*(m3*l3+m5*r3)*(t3DDot*cos(t1-t3)+t3Dot*t3Dot*sin(t1-t3))-m7/(M+m7)*(m4*l4)*(t4DDot*cos(t1-t4)+t4Dot*t4Dot*sin(t1-t4))-m7/(M+m7)*(m5*l5)*(t5DDot*cos(t1-t5)+t5Dot*t5Dot*sin(t1-t5))-M*g*cos(a2)*sin(t1-a2)))/((M+m6+m7+((M+m7)*cos(a1-a2)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(-(M+m7)*cos(a1-a2)+M*cos(a2-t1)*cos(a1-t1))+((M+m7)*cos(a1-t1)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(-M*cos(a1-t1)+M*cos(a2-t1)*cos(a1-a2)))*d1);

    double b =(M*(-d1*a*sin(a1-t1)*sin(a2-t1)-d1*a1Dot*a1Dot*cos(a1-t1)*sin(a2-t1)-d2*a2Dot*a2Dot*cos(a2-t1)*sin(a2-t1))-(m2*l2+m4*r2)*(t2DDot*sin(t2-t1)*sin(a2-t1)+t2Dot*t2Dot*cos(t2-t1)*sin(a2-t1))-(m3*l3+m5*r3)*(t3DDot*sin(t3-t1)*sin(a2-t1)+t3Dot*t3Dot*cos(t3-t1)*sin(a2-t1))-(m4*l4)*(t4DDot*sin(t4-t1)*sin(a2-t1)+t4Dot*t4Dot*cos(t4-t1)*sin(a2-t1))-(m5*l5)*(t5DDot*sin(t5-t1)*sin(a2-t1)+t5Dot*t5Dot*cos(t5-t1)*sin(a2-t1))+M*g*cos(t1)*sin(t1-a2)+M*l1*t1Dot*t1Dot*sin(t1-a2)-m7*d1*(a*cos(a1-a2)-a1Dot*a1Dot*sin(a1-a2))-m7*g*sin(a2))/((M+m7-M*cos(a2-t1)*cos(a2-t1))*d2) -2*c*a2Dot;
    return b;
}

//Differential equation for thetaDot3
double::PendulumSystem::thetaDot3(double omega3)
{
    return omega3;
}

//Differential equation for omegaDot3
double::PendulumSystem::omegaDot3(double d1, double d2, double l1,double l2, double l3,double l4, double l5, double r2,double r3, double a1 ,double a2,double t1,double a1Dot, double a2Dot, double t1Dot, double c,double m1, double m2, double m3, double m4, double m5, double m6, double m7, double t2, double t3, double t4, double t5, double t2Dot, double t3Dot, double t4Dot, double t5Dot, double t2DDot, double t3DDot, double t4DDot, double t5DDot)
{
    double M=m1+m2+m3+m4+m5;
    double g = 9.807;

    double a = (-M*(d2*a2Dot*a2Dot*sin(a1-a2)+l1*t1Dot*t1Dot*sin(a1-t1))-(m2*l2+m4*r2)*(t2DDot*cos(a1-t2)+t2Dot*t2Dot*sin(a1-t2))-(m3*l3+m5*r3)*(t3DDot*cos(a1-t3)+t3Dot*t3Dot*sin(a1-t3))-(m4*l4)*(t4DDot*cos(a1-t4)+t4Dot*t4Dot*sin(a1-t4))-(m5*l5)*(t5DDot*cos(a1-t5)+t5Dot*t5Dot*sin(a1-t5))-(M+m6+m7)*g*sin(a1)-m7*d2*a2Dot*a2Dot*sin(a1-a2)-((M+m7)*cos(a1-a2)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(
                    M*(-d1*a1Dot*a1Dot*cos(a1-t1)*sin(a2-t1)-d2*a2Dot*a2Dot*cos(a2-t1)*sin(a2-t1))-(m2*l2+m4*r2)*(t2DDot*sin(t2-t1)*sin(a2-t1)+t2Dot*t2Dot*cos(t2-t1)*sin(a2-t1))-(m3*l3+m5*r3)*(t3DDot*sin(t3-t1)*sin(a2-t1)+t3Dot*t3Dot*cos(t3-t1)*sin(a2-t1))-(m4*l4)*(t4DDot*sin(t4-t1)*sin(a2-t1)+t4Dot*t4Dot*cos(t4-t1)*sin(a2-t1))-(m5*l5)*(t5DDot*sin(t5-t1)*sin(a2-t1)+t5Dot*t5Dot*cos(t5-t1)*sin(a2-t1))
                    +M*g*cos(t1)*sin(t1-a2)+M*l1*t1Dot*t1Dot*sin(t1-a2)+m7*d1*a1Dot*a1Dot*sin(a1-a2)-m7*g*sin(a2))-((M+m7)*cos(a1-t1)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(M*d1*a1Dot*a1Dot*cos(a1-a2)*sin(a2-t1)+(M*M/(M+m7))*(l1*t1Dot*t1Dot*cos(a2-t1)*sin(a2-t1))-(M*sin(t1-a2))/(M+m7)*((m2*l2+m4*r2)*(t2DDot*sin(t2-a2)+t2Dot*t2Dot*cos(t2-a2))+(m3*l3+m5*r3)*(t3DDot*sin(t3-a2)+t3Dot*t3Dot*cos(t3-a2))+(m4*l4)*(t4DDot*sin(t4-a2)+t4Dot*t4Dot*cos(t4-a2))+(m5*l5)*(t5DDot*sin(t5-a2)+t5Dot*t5Dot*cos(t5-a2)))
                    +M*d2*a2Dot*a2Dot*sin(a2-t1)-m7/(M+m7)*(m2*l2+m4*r2)*(t2DDot*cos(t1-t2)+t2Dot*t2Dot*sin(t1-t2))-m7/(M+m7)*(m3*l3+m5*r3)*(t3DDot*cos(t1-t3)+t3Dot*t3Dot*sin(t1-t3))-m7/(M+m7)*(m4*l4)*(t4DDot*cos(t1-t4)+t4Dot*t4Dot*sin(t1-t4))-m7/(M+m7)*(m5*l5)*(t5DDot*cos(t1-t5)+t5Dot*t5Dot*sin(t1-t5))-M*g*cos(a2)*sin(t1-a2)))/((M+m6+m7+((M+m7)*cos(a1-a2)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(-(M+m7)*cos(a1-a2)+M*cos(a2-t1)*cos(a1-t1))+((M+m7)*cos(a1-t1)/(m7+M*sin(a2-t1)*sin(a2-t1)))*(-M*cos(a1-t1)+M*cos(a2-t1)*cos(a1-a2)))*d1);

    return (M*(d1*a*sin(a1-a2)*sin(a2-t1)+d1*a1Dot*a1Dot*cos(a1-a2)*sin(a2-t1))+((M*M)/(M+m7))*(l1*t1Dot*t1Dot*cos(a2-t1)*sin(a2-t1))-(M*sin(t1-a2))/(M+m7)*((m2*l2+m4*r2)*(t2DDot*sin(t2-a2)+t2Dot*t2Dot*cos(t2-a2))+(m3*l3+m5*r3)*(t3DDot*sin(t3-a2)+t3Dot*t3Dot*cos(t3-a2))+(m4*l4)*(t4DDot*sin(t4-a2)+t4Dot*t4Dot*cos(t4-a2))+(m5*l5)*(t5DDot*sin(t5-a2)+t5Dot*t5Dot*cos(t5-a2)))-M*g*cos(a2)*sin(t1-a2)+M*d2*a2Dot*a2Dot*sin(a2-t1)-(m7/(M+m7))*((m2*l2+m4*r2)*(t2DDot*cos(t1-t2)+t2Dot*t2Dot*sin(t1-t2))+(m3*l3+m5*r3)*(t3DDot*cos(t1-t3)+t3Dot*t3Dot*sin(t1-t3))+(m4*l4)*(t4DDot*cos(t1-t4)+t4Dot*t4Dot*sin(t1-t4))+(m5*l5)*(t5DDot*cos(t1-t5)+t5Dot*t5Dot*sin(t1-t5))))/((M-((M*M*cos(a2-t1)*cos(a2-t1))/(M+m7)))*l1) -2*c*t1Dot;
}

//Initializing variables
double a4 = 0; //Acceleration of alpha1
double t = 0; //Time elapsed
double time = 0; //Time since last occurrence of alpha1 = 0
double t2 = 0;
double t3 = 0;
double t4 = 0;
double t5 = 0;
double t2Dot = 0;
double t3Dot = 0;
double t4Dot = 0;
double t5Dot = 0;
double t2DDot = 0;
double t3DDot = 0;
double t4DDot = 0;
double t5DDot = 0;
double theta4 = 0; //alpha1 from previous step
int j = 0; //Counting variable used to record data after certain time intervals
std::ofstream outFile("DHDFlail.txt");

//Simulates a time step
void PendulumSystem::Simulate(double h) {

    int NPendulums = fPendulums.size(); //Number of pendulums

    for (int iComponent = 0; iComponent < NPendulums; ++iComponent) {

        if (t == 0) outFile << "Time" << "\t" << "Alpha1" << "\t" << "Alpha2" << "\t" << "Theta1" << std::endl;

        double iTheta = fPendulums[iComponent]->GetTheta(); //Rewriting Displacement
        double theta2 = fPendulums[iComponent]->GetTheta2(); //Rewriting Displacement
        double omega = fPendulums[iComponent]->GetThetaDot(); //Rewriting velocity
        double omega2 = fPendulums[iComponent]->GetThetaDot2(); //Rewriting velocity
        double theta3 = fPendulums[iComponent]->GetTheta3(); //Rewriting Displacement
        double omega3 = fPendulums[iComponent]->GetThetaDot3(); //Rewriting velocity

        //Records data for up to t = 800s
        if (j % 10 == 0 && t <= 800) outFile << t << "\t" << iTheta << "\t" << theta2 << "\t" << theta3 << std::endl;
        if (j % 1000 == 0) std::cout << t << "\t" << iTheta << std::endl;


        //Lengths of different parts of the system
        double d1 = 1.51;
        double d2 = 0.12;
        double l1 = 0.185;
        double l2 = 0.2115*2/3.0;
        double l3 = 0.1*0.5;
        double l4 = 0.11441*0.5;
        double l5 = 0.14809*2/3.0;
        double r2 = 0.2115;
        double r3 = 0.1;

        //Maximum displacement of robot parts from average positions
        double A = 0.2394;
        double B = 0.8328;
        double C = 0.826214;

        double k = 0.10; //Characteristic speed of robot movement
        double f = 2; //Scales the magnitude of the robot movement
        double v = 0.3; //Time delay for robot movement after alpha1 = 0

        //Masses of different components
        double m1 = 0.42068;
        double m2 = 2.20676;
        double m3 = 0.77936;
        double m4 = 0.68375;
        double m5 = 1.21484;
        double m6 = 1.291;
        double m7 = 1.082;
        double c = fPendulums[iComponent]->GetGamma(); //viscous damping coefficient

        //Runge Kutta parameters
        double omega1 = 0, omega02 = 0, omega03 = 0; //omega at intervals in Runge Kutta calculation
        double wk1 = 0,wk2 = 0,wk3 = 0,wk4 = 0; // k values for omega
        double theta1 = 0, theta02 = 0, theta03 = 0; //theta at intervals in Runge Kutta calculation
        double ok1 = 0,ok2 = 0,ok3 = 0,ok4 = 0; // k values for theta
        double oError = 0, wError = 0; //errors on theta and omega

        double omega21 = 0, omega22 = 0, omega23 = 0; //omega2 at intervals in Runge Kutta calculation
        double wk12 = 0,wk22 = 0,wk32 = 0,wk42 = 0; // k values for omega2
        double theta21 = 0, theta22 = 0, theta23 = 0; //theta2 at intervals in Runge Kutta calculation
        double ok12 = 0,ok22 = 0,ok32 = 0,ok42 = 0; // k values for theta2
        double oError2 = 0, wError2 = 0; //errors on theta2 and omega2.

        double omega31 = 0, omega32 = 0, omega33 = 0; //omega3 at intervals in Runge Kutta calculation
        double wk13 = 0,wk23 = 0,wk33 = 0,wk43 = 0; // k values for omega3
        double theta31 = 0, theta32 = 0, theta33 = 0; //theta3 at intervals in Runge Kutta calculation
        double ok13 = 0,ok23 = 0,ok33 = 0,ok43 = 0; // k values for theta3
        double oError3 = 0, wError3 = 0; //errors on theta3 and omega3.

        //Initial positions of robot limbs
        if(t==0){
            t2=iTheta-2.3209+A;
            t3=iTheta+1.57;
            t4=iTheta-2.2424+B;
            t5=iTheta+0.836686+C;
            t2Dot=omega;
            t3Dot=omega;
            t4Dot=omega;
            t5Dot=omega;
            t2DDot=a4;
            t3DDot=a4;
            t4DDot=a4;
            t5DDot=a4;
        }

        //Positions of robot limbs reset as swing moves through lowest point of swing.
        if (iTheta < 0 && theta4 >= 0 && t > 0.1) {
            time = 0;
            t2=iTheta-2.3209-A;
            t3=iTheta+1.57;
            t4=iTheta-2.2424-B;
            t5=iTheta+0.836686-C;
            t2Dot=omega;
            t3Dot=omega;
            t4Dot=omega;
            t5Dot=omega;
            t2DDot=a4;
            t3DDot=a4;
            t4DDot=a4;
            t5DDot=a4;
        }

        else if (iTheta >= 0 && theta4 < 0 && t > 0.1){
            time = 0;
            t2=iTheta-2.3209+A;
            t3=iTheta+1.57;
            t4=iTheta-2.2424+B;
            t5=iTheta+0.836686+C;
            t2Dot=omega;
            t3Dot=omega;
            t4Dot=omega;
            t5Dot=omega;
            t2DDot=a4;
            t3DDot=a4;
            t4DDot=a4;
            t5DDot=a4;
        }

        //Robot movement follows a step function approximation
        if (iTheta < 0 && t > 0.1){
            if (t2 < theta4-2.3209-0.0001+A){
                t2=iTheta-2.3209-A+f*A/(1+exp((-time+v)/k));
                t3=iTheta+1.57;
                t4=iTheta-2.2424-B+f*B/(1+exp((-time+v)/k));
                t5=iTheta+0.836686-C+f*C/(1+exp((-time+v)/k));
                t2Dot=omega+f*A/(2*k+k*cosh((time-v)/k));
                t3Dot=omega;
                t4Dot=omega+f*B/(2*k+k*cosh((time-v)/k));
                t5Dot=omega+f*C/(2*k+k*cosh((time-v)/k));
                t2DDot=a4+f*A*4*sinh((time-v)/k)/(k*k*(2*cosh((time-v)/k)+4)*(2*cosh((time-v)/k)+4));
                t3DDot=a4;
                t4DDot=a4+f*B*4*sinh((time-v)/k)/(k*k*(2*cosh((time-v)/k)+4)*(2*cosh((time-v)/k)+4));
                t5DDot=a4+f*C*4*sinh((time-v)/k)/(k*k*(2*cosh((time-v)/k)+4)*(2*cosh((time-v)/k)+4));
            }
            else {
                t2=iTheta-2.3209+A;
                t3=iTheta+1.57;
                t4=iTheta-2.2424+B;
                t5=iTheta+0.836686+C;
                t2Dot=omega;
                t3Dot=omega;
                t4Dot=omega;
                t5Dot=omega;
                t2DDot=a4;
                t3DDot=a4;
                t4DDot=a4;
                t5DDot=a4;
            }
        }
        else if (iTheta >= 0 || t <= 0.1){
            if (t2 > theta4-2.3209+0.0001-A){
                t2=iTheta-2.3209+A-f*A/(1+exp((-time+v)/k));
                t3=iTheta+1.57;
                t4=iTheta-2.2424+B-f*B/(1+exp((-time+v)/k));
                t5=iTheta+0.836686+C-f*C/(1+exp((-time+v)/k));
                t2Dot=omega-f*A/(2*k+k*cosh((time-v)/k));
                t3Dot=omega;
                t4Dot=omega-f*B/(2*k+k*cosh((time-v)/k));
                t5Dot=omega-f*C/(2*k+k*cosh((time-v)/k));
                t2DDot=a4-f*A*4*sinh((time-v)/k)/(k*k*(2*cosh((time-v)/k)+4)*(2*cosh((time-v)/k)+4));
                t3DDot=a4;
                t4DDot=a4-f*B*4*sinh((time-v)/k)/(k*k*(2*cosh((time-v)/k)+4)*(2*cosh((time-v)/k)+4));
                t5DDot=a4-f*C*4*sinh((time-v)/k)/(k*k*(2*cosh((time-v)/k)+4)*(2*cosh((time-v)/k)+4));
            }
            else {
                t2=iTheta-2.3209-A;
                t3=iTheta+1.57;
                t4=iTheta-2.2424-B;
                t5=iTheta+0.836686-C;
                t2Dot=omega;
                t3Dot=omega;
                t4Dot=omega;
                t5Dot=omega;
                t2DDot=a4;
                t3DDot=a4;
                t4DDot=a4;
                t5DDot=a4;
            }
        }

        a4 = omegaDot(d1, d2, l1, l2, l3, l4, l5, r2, r3, iTheta , theta2, theta3, omega, omega2, omega3, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);

        theta4 = iTheta;

        //step part 1
        ok1 = thetaDot(omega);
        wk1 = omegaDot(d1, d2, l1, l2, l3, l4, l5, r2, r3, iTheta , theta2, theta3, omega, omega2, omega3, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta1 = iTheta + ok1 * (h/2);
        omega1 = omega + wk1 * (h/2);

        ok12 = thetaDot2(omega2);
        wk12 = omegaDot2(d1, d2, l1, l2, l3, l4, l5, r2, r3, iTheta , theta2, theta3, omega, omega2, omega3, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta21 = theta2 + ok12 * (h/2);
        omega21 = omega2 + wk12 * (h/2);

        ok13 = thetaDot3(omega3);
        wk13 = omegaDot3(d1, d2, l1, l2, l3, l4, l5, r2, r3, iTheta , theta2, theta3, omega, omega2, omega3, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta31 = theta3 + ok13 * (h/2);
        omega31 = omega3 + wk13 * (h/2);

        //step part 2
        ok2 = thetaDot(omega1);
        wk2 = omegaDot(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta1, theta21, theta31, omega1, omega21, omega31, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta02 = iTheta + ok2 * (h/2);
        omega02 = omega + wk2 * (h/2);

        ok22 = thetaDot2(omega21);
        wk22 = omegaDot2(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta1, theta21, theta31, omega1, omega21, omega31, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta22 = theta2 + ok22 * (h/2);
        omega22 = omega2 + wk22 * (h/2);

        ok23 = thetaDot3(omega31);
        wk23 = omegaDot3(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta1, theta21, theta31, omega1, omega21, omega31, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta32 = theta3 + ok23 * (h/2);
        omega32 = omega3 + wk23 * (h/2);

        //step part 3
        ok3 = thetaDot(omega02);
        wk3 = omegaDot(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta02, theta22, theta32, omega02, omega22, omega32, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta03 = iTheta + ok3 * (h);
        omega03 = omega + wk3 * (h);

        ok32 = thetaDot2(omega22);
        wk32 = omegaDot2(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta02, theta22, theta32, omega02, omega22, omega32, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta23 = theta2 + ok32 * (h);
        omega23 = omega2 + wk32 * (h);

        ok33 = thetaDot3(omega32);
        wk33 = omegaDot3(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta02, theta22, theta32, omega02, omega22, omega32, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);
        theta33 = theta3 + ok33 * (h);
        omega33 = omega3 + wk33 * (h);

        //step part 4
        ok4 = thetaDot(omega03);
        wk4 = omegaDot(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta03 , theta23, theta33, omega03, omega23, omega33, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);

        ok42 = thetaDot2(omega23);
        wk42 = omegaDot2(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta03 , theta23, theta33, omega03, omega23, omega33, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);

        ok43 = thetaDot3(omega33);
        wk43 = omegaDot3(d1, d2, l1, l2, l3, l4, l5, r2, r3, theta03 , theta23, theta33, omega03, omega23, omega33, c, m1, m2, m3, m4, m5, m6, m7, t2, t3, t4, t5, t2Dot, t3Dot, t4Dot, t5Dot, t2DDot, t3DDot, t4DDot, t5DDot);

        //calculates new values of theta and omega
        iTheta += (ok1 + 2*ok2 + 2*ok3 + ok4) * (h/6);
        omega += (wk1 + 2*wk2 + 2*wk3 + wk4) * (h/6);

        //calculates errors on theta and omega
        wError += (8.0/15.0) * pow((wk3 -wk2), 3.0) * 1/(pow((wk4-wk1),2.0));
        oError += (8.0/15.0) * pow((ok3 -ok2), 3.0) * 1/(pow((ok4-ok1),2.0));

        //calculates new values of theta2 and omega2
        theta2 += (ok12 + 2*ok22 + 2*ok32 + ok42) * (h/6);
        omega2 += (wk12 + 2*wk22 + 2*wk32 + wk42) * (h/6);

        //calculates errors on theta2 and omega2
        wError2 += (8.0/15.0) * pow((wk32 -wk22), 3.0) * 1/(pow((wk42-wk12),2.0));
        oError2 += (8.0/15.0) * pow((ok32 -ok22), 3.0) * 1/(pow((ok42-ok12),2.0));

        //calculates new values of theta3 and omega3
        theta3 += (ok13 + 2*ok23 + 2*ok33 + ok43) * (h/6);
        omega3 += (wk13 + 2*wk23 + 2*wk33 + wk43) * (h/6);

        //calculates errors on theta3 and omega3
        wError3 += (8.0/15.0) * pow((wk33 -wk23), 3.0) * 1/(pow((wk43-wk13),2.0));
        oError3 += (8.0/15.0) * pow((ok33 -ok23), 3.0) * 1/(pow((ok43-ok13),2.0));

        //Setting new values for angles
        fPendulums[iComponent]->setTheta(iTheta);
        fPendulums[iComponent]->setThetaDot(omega);
        fPendulums[iComponent]->setTheta2(theta2);
        fPendulums[iComponent]->setThetaDot2(omega2);
        fPendulums[iComponent]->setTheta3(theta3);
        fPendulums[iComponent]->setThetaDot3(omega3);
    }

    t = t + h;
    time = time + h;
    j = j + 1;
}
