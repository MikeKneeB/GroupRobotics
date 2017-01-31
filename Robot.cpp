#include "Robot.h"


/*
 * Left to do:
 * + Calculate position of main robot body part from angle
 * + Calculate instantaneous velocity from initial and final position
 * + Same for instantaneous acceleration
 * + Do the same for the move(t) function in the BodyPart class
 *
 */


Robot::Robot(double mass,double angle,double maxAngle,double length):
    BodyPart::BodyPart(0,mass,angle,maxAngle,length)
{

}

Robot::~Robot(){
    for(unsigned int i=0;i<fParts.size();i++){
        delete(fParts[i]);
    }
}

void Robot::addBodyPart(BodyPart *part){
    fParts.push_back(part);
}

void Robot::move(double t){
    setAngle(getMaxAngle()*sin(t));
    setPosition(TwoVector(getLength()*sin(getAngle()),getLength()*cos(getAngle())));

    BodyPart* part;
    for(unsigned int i=0;i<fParts.size();i++){
        part=fParts[i];
        part->move(t);
    }
}
