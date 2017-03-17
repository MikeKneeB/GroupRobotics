#ifndef CHARGEDPENDULUM_H
#define CHARGEDPENDULUM_H

#include "Pendulum.h"

//This class is for a specific type of pendulum (charged), and is inherited from the class pendulum
//Note that charge is in Coulombs so value should be very small

class ChargedPendulum: public Pendulum {

public:
    //Constructors
    ChargedPendulum();
    ChargedPendulum(std::string name, double mass, double length, double theta, double thetaDot, double theta2, double thetaDot2, double theta3, double thetaDot3, double z, double gamma);
    virtual ~ChargedPendulum(); //Destructor

    //Access methods - Methods that were purely virtual in Pendulum will now be implemented
    inline double GetCharge(); //Returns charge of pendulum
    inline void setCharge(double charge);
    inline int GetId(); //Used to identify pendulum type
    //Used to change the properties of a pendulum from the control window
    //This is polymorphic, as the method is slightly different for the heavy pendulum class
    void SetValue(std::string VariableName, std::string Value);

private:
    double fCharge; //Charge of pendulum in Coulombs
};

//In this class, pendulums are charged
inline double ChargedPendulum::GetCharge() {return fCharge;}
inline void ChargedPendulum::setCharge(double charge) {fCharge = charge;}
inline int ChargedPendulum::GetId() {return 2;} //ID of charged pendulum class

#endif // CHARGEDPENDULUM_H
