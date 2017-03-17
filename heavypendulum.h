#ifndef HEAVYPENDULUM_H
#define HEAVYPENDULUM_H

#include "Pendulum.h"

//This class is for a specific type of pendulum (can be attracted to other pendulums due to gravity)
//It is inherited from the class pendulum. Note that mass must be very large for noticeable attraction.

class HeavyPendulum: public Pendulum {

public:
    //Constructors
    HeavyPendulum();
    HeavyPendulum(std::string name, double mass, double length, double theta, double thetaDot, double theta2, double thetaDot2, double theta3, double thetaDot3, double z, double gamma);
    virtual ~HeavyPendulum(); //Destructor

    //Access methods - Methods that were purely virtual in Pendulum will now be implemented
    inline double GetCharge();
    inline void setCharge(double);
    inline int GetId(); //Used to identify pendulum type
    //Used to change the properties of a pendulum from the control window
    //This is polymorphic, as the method is slightly different for the charged pendulum class
    void SetValue(std::string VariableName, std::string Value);
};

//Charge does not apply to this class - It is returned as zero
inline double HeavyPendulum::GetCharge() {return 0;}
inline void HeavyPendulum::setCharge(double) {}
inline int HeavyPendulum::GetId() {return 1;} //ID of heavy pendulum class

#endif // HEAVYPENDULUM_H
