#ifndef PENDULUMSYSTEM_H
#define PENDULUMSYSTEM_H

#include <vector>

#include "ChargedPendulum.h"
#include "HeavyPendulum.h"

//This class is used to create and simulate a system of pendulums

class PendulumSystem{

public:
    //Constructor
    PendulumSystem();
    virtual ~PendulumSystem(); //Destructor

    //Adds a pendulum to an array of pendulums
    void AddPendulum(Pendulum * pendulum) {
        fPendulums.push_back(pendulum);
    }
    //Returns size of array of pendulums
    int GetNPendulums() {
        return fPendulums.size();
    }
    //Returns a pendulum from the array
    Pendulum * GetPendulum(int i){
        return fPendulums[i];
    }

    void Simulate(double dt); //Simulates the system of pendulums
    Pendulum* FindPendulum(std::string Name); //Used to display pendulum properties in the GUI
    inline void ToggleDisable(); //Disables/Enables interactions between pendulums

    double thetaDot(double omega);
    double omegaDot(double d1, double d2, double l1,double l2, double l3,double l4, double l5, double r2,double r3, double a1 ,double a2,double t1,double a1Dot, double a2Dot, double t1Dot, double c,double m1, double m2, double m3, double m4, double m5, double m6, double m7, double t2, double t3, double t4, double t5, double t2Dot, double t3Dot, double t4Dot, double t5Dot, double t2DDot, double t3DDot, double t4DDot, double t5DDot);
    double thetaDot2(double omega2);
    double omegaDot2(double d1, double d2, double l1,double l2, double l3,double l4, double l5, double r2,double r3, double a1 ,double a2,double t1,double a1Dot, double a2Dot, double t1Dot, double c,double m1, double m2, double m3, double m4, double m5, double m6, double m7, double t2, double t3, double t4, double t5, double t2Dot, double t3Dot, double t4Dot, double t5Dot, double t2DDot, double t3DDot, double t4DDot, double t5DDot);
    double thetaDot3(double omega3);
    double omegaDot3(double d1, double d2, double l1,double l2, double l3,double l4, double l5, double r2,double r3, double a1 ,double a2,double t1,double a1Dot, double a2Dot, double t1Dot, double c,double m1, double m2, double m3, double m4, double m5, double m6, double m7, double t2, double t3, double t4, double t5, double t2Dot, double t3Dot, double t4Dot, double t5Dot, double t2DDot, double t3DDot, double t4DDot, double t5DDot);


private:
    std::vector<Pendulum*> fPendulums; //Array of pendulums
    bool fDisable; //Disables/Enables interactions between pendulums

};

//Toggles interactions on/off
inline void PendulumSystem::ToggleDisable() {fDisable = !fDisable;}

#endif // PENDULUMSYSTEM_H
