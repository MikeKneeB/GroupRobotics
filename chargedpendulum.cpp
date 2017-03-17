#include <sstream>

#include "ChargedPendulum.h"

//Constructors
ChargedPendulum::ChargedPendulum() {}
ChargedPendulum::ChargedPendulum(std::string name, double mass, double length, double theta, double thetaDot, double theta2, double thetaDot2, double theta3, double thetaDot3, double z, double gamma)
:Pendulum(name, mass, length, theta, thetaDot, theta2, thetaDot2, theta3, thetaDot3, z, gamma) {
    fCharge = 0; //Charge initialised as zero
}
ChargedPendulum::~ChargedPendulum() {} //Destructor

//Used to change the properties of a pendulum from the control window
//This is polymorphic, as the method is slightly different for the heavy pendulum class
void ChargedPendulum::SetValue(std::string VariableName, std::string Value) {
    double Value2;
    //Stringstream used to convert the value from string to double
    std::stringstream(Value) >> Value2;

    //The input value changes the value of the variable corresponding to the chosen input field in the control window
    if(VariableName.compare("Mass") == 0)
        setMass(Value2);
    else if(VariableName.compare("Length") == 0)
        setLength(Value2);
    else if(VariableName.compare("Theta") == 0)
        setTheta(Value2);
    else if(VariableName.compare("ThetaDot") == 0)
        setThetaDot(Value2);
    else if(VariableName.compare("Z") == 0)
        setZ(Value2);
    else if(VariableName.compare("Gamma") == 0)
        setGamma(Value2);
    else if(VariableName.compare("Charge") == 0)
        fCharge = Value2;
}

