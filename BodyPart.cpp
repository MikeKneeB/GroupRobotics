#include "BodyPart.h"

#include <cmath>

BodyPart::BodyPart(BodyPart* parent,double mass,double angle,double maxAngle,double length):
    fParent(parent),
    fMass(mass),
    fAngle(angle),
    fMaxAngle(maxAngle),
    fLength(length)
{
    fPosition=TwoVector(0,0);
    fVelocity=TwoVector(0,0);
}

BodyPart::~BodyPart(){}

void BodyPart::move(double t){
    TwoVector iPos=fPosition;
    fAngle=fMaxAngle*sin(t);
}
