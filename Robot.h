#ifndef ROBOT_H
#define ROBOT_H

#include "BodyPart.h"
#include <vector>

class Robot: public BodyPart
{
public:
    Robot(double mass,double angle,double maxAngle,double length);
    virtual ~Robot();

    void addBodyPart(BodyPart* part);

    inline double getCOMMass();
    inline TwoVector getCOMPosition();
    inline TwoVector getCOMVelocity();
    inline TwoVector getCOMAcceleration();

    virtual void move(double t);
private:
    std::vector<BodyPart*>fParts;
};

inline double Robot::getCOMMass(){
    double total=getMass();
    BodyPart* part;
    for(unsigned int i=0;i<fParts.size();i++){
        part=fParts[i];
        total+=part->getMass();
    }
    return total;
}
inline TwoVector Robot::getCOMPosition(){
    TwoVector total=getPosition()*getMass();
    BodyPart* part;
    for(unsigned int i=0;i<fParts.size();i++){
        part=fParts[i];
        total+=(part->getPosition()*part->getMass());
    }
    return total/getCOMMass();
}
inline TwoVector Robot::getCOMVelocity(){
    TwoVector total =getVelocity()*getMass();
    BodyPart* part;
    for(unsigned int i=0;i<fParts.size();i++){
        part=fParts[i];
        total+=(part->getVelocity()*part->getMass());
    }
    return total/getCOMMass();
}
inline TwoVector Robot::getCOMAcceleration(){
    TwoVector total =getAcceleration()*getMass();
    BodyPart* part;
    for(unsigned int i=0;i<fParts.size();i++){
        part=fParts[i];
        total+=(part->getAcceleration()*part->getMass());
    }
    return total/getCOMMass();
}

#endif // ROBOT_H
