#include <cmath>
#include <iostream>
#include <cmath>

#include "Pendulum.h"

//Default constructor
Pendulum::Pendulum() {
}
//Main constructor
Pendulum::Pendulum(std::string name, double mass, double length, double theta, double thetaDot, double theta2, double thetaDot2, double theta3, double thetaDot3, double z, double gamma)
: fName(name), fMass(mass), fLength(length), fTheta(theta), fThetaDot(thetaDot), fTheta2(theta2), fThetaDot2(thetaDot2), fTheta3(theta3), fThetaDot3(thetaDot3), fZ(z), fGamma(gamma) {}
Pendulum::~Pendulum() {} //Destructor

//This is used to find the angle Theta and angular velocity ThetaDot of the pendulum
//after an additional time interval dt. It is calculated numerically.
void Pendulum::Simulate(double dt, double Acceleration) {
    fTheta += 0.5*Acceleration*dt*dt + fThetaDot*dt;
    fThetaDot += Acceleration*dt;
}

//Angular Acceleration - a is the additional acceleration due to interactions with other pendulums
void Pendulum::setThetaDoubleDot(double a) {
    //double const g = 9.81; //m/s, acceleration of free fall.
    //fThetaDoubleDot = -fGamma*fThetaDot -g*sin(fTheta)/fLength + a;
    //double l = 1;
    //double A = 0.1;
    //double Omega = 6;
    //double Phi = 0;
    fThetaDoubleDot = a;
}

//(Horizontal) X Displacement
double Pendulum::X() const {
    return fLength * sin(fTheta);
}

//(Vertical) Y Displacement
double Pendulum::Y() const {
    return -fLength*cos(fTheta);
}

//Threevector Position
const ThreeVector Pendulum::GetPosition() const {
    return ThreeVector (X(), Y(), Z());
}
