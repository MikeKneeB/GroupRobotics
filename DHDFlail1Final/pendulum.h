//Written by Daniel Corless

#ifndef PENDULUM_H
#define PENDULUM_H

#include <string>

#include "ThreeVector.h"

//This is a class for a simple pendulum. It contains the variables name, mass in kg, length in metres, angular position (theta),
//angular velocity (thetaDot), hinge angles theta2 and theta3, hinge angular velocities theta2Dot and theta3Dot, position perpendicular to plane of motion (z) in metres and damping coefficient (gamma).
//Note that the z coordinate is not currently being used.
class Pendulum
{
public:
    //Default Constructor
    Pendulum();
    //Main Constructor
    Pendulum(std::string name, double mass, double length, double theta, double thetaDot, double theta2, double thetaDot2, double theta3, double thetaDot3, double z, double gamma);
    virtual ~Pendulum(); //Destructor

    //Access methods for each variable
    inline const std::string& GetName() const;
    inline double GetMass() const;
    inline double GetLength() const;
    inline double Z() const;
    inline double GetGamma() const; 
    inline double GetTheta() const;
    inline double GetThetaDot() const;
    inline double GetTheta2() const;
    inline double GetThetaDot2() const;
    inline double GetTheta3() const;
    inline double GetThetaDot3() const;
    inline double GetThetaDoubleDot() const; //Angular acceleration of pendulum
    double X() const; //(Horizontal) X Displacement
    double Y() const; //(Vertical) Y Displacement
    const ThreeVector GetPosition() const; //Threevector position

    //Simulates a step of time dt of the motion of the pendulum
    void Simulate(double dt, double Acceleration);

    //Methods used to set variables
    inline void setMass(double mass);
    inline void setLength(double length);
    inline void setTheta(double theta);
    inline void setThetaDot(double thetadot);
    inline void setTheta2(double theta2);
    inline void setThetaDot2(double thetadot2);
    inline void setTheta3(double theta3);
    inline void setThetaDot3(double thetadot3);
    inline void setZ(double z);
    inline void setGamma(double gamma);
    void setThetaDoubleDot(double a);

private:
    //Fundamental variables of pendulum
    std::string fName;
    double fMass;
    double fLength;
    double fTheta;
    double fThetaDot;
    double fTheta2;
    double fThetaDot2;
    double fTheta3;
    double fThetaDot3;
    double fZ;
    double fGamma;
    double fThetaDoubleDot; //Angular acceleration of pendulum
};

//Defining previously declared methods
inline const std::string& Pendulum::GetName() const {
    return fName;
}

inline double Pendulum::GetMass() const {
    return fMass;
}

inline double Pendulum::GetLength() const {
    return fLength;
}

inline double Pendulum::GetTheta() const {
    return fTheta;
}

inline double Pendulum::GetThetaDot() const {
    return fThetaDot;
}

inline double Pendulum::GetTheta2() const {
    return fTheta2;
}

inline double Pendulum::GetThetaDot2() const {
    return fThetaDot2;
}

inline double Pendulum::GetTheta3() const {
    return fTheta3;
}

inline double Pendulum::GetThetaDot3() const {
    return fThetaDot3;
}

inline double Pendulum::Z() const {
    return fZ;
}

inline double Pendulum::GetGamma() const {
    return fGamma;
}

inline double Pendulum::GetThetaDoubleDot() const {
    return fThetaDoubleDot;
}

inline void Pendulum::setMass(double mass) {
    fMass = mass;
}

inline void Pendulum::setLength(double length) {
    fLength = length;
}

inline void Pendulum::setTheta(double theta) {
    fTheta = theta;
}

inline void Pendulum::setThetaDot(double thetadot) {
    fThetaDot = thetadot;
}

inline void Pendulum::setTheta2(double theta2) {
    fTheta2 = theta2;
}

inline void Pendulum::setThetaDot2(double thetadot2) {
    fThetaDot2 = thetadot2;
}

inline void Pendulum::setTheta3(double theta3) {
    fTheta3 = theta3;
}

inline void Pendulum::setThetaDot3(double thetadot3) {
    fThetaDot3 = thetadot3;
}

inline void Pendulum::setZ(double z) {
    fZ = z;
}

inline void Pendulum::setGamma(double gamma) {
    fGamma = gamma;
}

#endif // PENDULUM_H
