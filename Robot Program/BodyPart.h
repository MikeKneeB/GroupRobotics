#ifndef BODYPART_H
#define BODYPART_H

#include "TwoVector.h"
#include <vector>
class BodyPart
{
public:
    BodyPart(BodyPart* parent, double mass, double angle, double length);
    virtual ~BodyPart();

    inline TwoVector getPosition();
    inline TwoVector getVelocity();
    inline TwoVector getAcceleration();
    inline double getMass();
    inline double getAngle();
    inline double getLength();

    inline void setPosition(TwoVector);
    inline void setVelocity(TwoVector);
    inline void setAcceleration(TwoVector);
    inline void setAngle(double);
    inline void addPart(BodyPart *part);

    virtual void move(double thetaF, double speed, double dt);

    //could not find a way to access otherwise, so made public
    BodyPart* fParent; // if no parent = 0
private:

    double fMass;
    double fAngle;
    TwoVector fPosition;
    TwoVector fVelocity;
    TwoVector fAcceleration;
    double fLength;
    std::vector<BodyPart*>fParts;
};

inline TwoVector BodyPart::getPosition(){return fPosition;}
inline TwoVector BodyPart::getVelocity(){return fVelocity;}
inline TwoVector BodyPart::getAcceleration(){return fAcceleration;}
inline double BodyPart::getMass(){return fMass;}
inline double BodyPart::getAngle(){return fAngle;}
inline double BodyPart::getLength(){return fLength;}

inline void BodyPart::setPosition(TwoVector tV){fPosition=tV;}
inline void BodyPart::setVelocity(TwoVector tV){fVelocity=tV;}
inline void BodyPart::setAcceleration(TwoVector tV){fAcceleration=tV;}
inline void BodyPart::setAngle(double angle){fAngle=angle;}
inline void BodyPart::addPart(BodyPart *part){fParts.push_back(part);}


#endif // BODYPART_H

