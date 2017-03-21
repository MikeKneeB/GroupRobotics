//Written by Daniel Corless

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

//Angular Acceleration - a is the additional acceleration due to interactions with other pendulums
void Pendulum::setThetaDoubleDot(double a) {
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
