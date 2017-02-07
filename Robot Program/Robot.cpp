#include "Robot.h"




Robot::Robot(TwoVector Position):
    //robot acts as a point mass
    BodyPart::BodyPart(0,0,0,0)
{
    //sets position of robot starting point - where robot is sitting on swing
    setPosition(Position);
}

Robot::~Robot(){
    for(unsigned int i=0;i<fParts.size();i++){
        delete(fParts[i]);
    }
}

void Robot::addBodyPart(BodyPart *part){
    //adds part to robot - part list
    fParts.push_back(part);
    //adds part to parent part list
    part->fParent->addPart(part);

    //sets position based on parent
    TwoVector parentPos = part->fParent->getPosition();
    double parentAngle = part->fParent->getAngle();
    TwoVector partPos = TwoVector(part->getLength()*sin(part->getAngle() + parentAngle),part->getLength()*cos(part->getAngle()+parentAngle)) + parentPos;
    part->setPosition(partPos);

}


