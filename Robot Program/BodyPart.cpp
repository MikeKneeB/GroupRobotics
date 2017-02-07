#include "BodyPart.h"

#include <cmath>
#include <QDebug>
BodyPart::BodyPart(BodyPart* parent,double mass,double angle,double length):
    fParent(parent),
    fMass(mass),
    fAngle(angle),
    fLength(length)
{
    fPosition=TwoVector(0,0);
    fVelocity=TwoVector(0,0);
}

BodyPart::~BodyPart(){}

void BodyPart::move(double thetaF, double speed, double dt){

    //Gets parent angle and current angle
    double parentAngle = this->fParent->getAngle();
    double thetaI = fAngle;

    //checks direction of movement - +x is "right"
    bool forwards = true;

    if (thetaI > thetaF){
        //moving backwards - "left"
        forwards = false;
    }

    //if moving forwards
    if (forwards == true){
    //if within range of current angle and desired angle
    if (fAngle <= thetaF  && fAngle >= thetaI){
    //step angle forwards
    fAngle += speed*dt;
}
    }
    else{
        //moving backwards
        //if within range of current angle and desired angle
        if (fAngle >= thetaF  && fAngle <= thetaI){
        //step angle backwards
        fAngle -= speed*dt;
    }

    }
    //sets new angle
    setAngle(fAngle);

    //Position Change
    TwoVector iPos=fPosition;
    TwoVector fPos = TwoVector(getLength()*sin(getAngle()+parentAngle),getLength()*cos(getAngle()+parentAngle)) + fParent->getPosition();
    setPosition(fPos);

    //Velocity Change
    TwoVector iVel = fVelocity;
   TwoVector fVel = (fPos-iPos)/dt;
    setVelocity(fVel);

    //Acceleration change
    TwoVector Acc = (fVel - iVel)/dt;
    setAcceleration(Acc);

    //move child body parts
    for (int i = 0; i < fParts.size(); i++)
    {
        BodyPart* part;
        part = fParts[i];
        part->move(part->getAngle(),0,0);
    }


}
