#ifndef BODYPART_H
#define BODYPART_H

#include "TwoVector.h"

class BodyPart
{
public:
    BodyPart(BodyPart* parent,double mass,double angle,double maxAngle,double length);
    virtual ~BodyPart();

    inline TwoVector getPosition();
    inline TwoVector getVelocity();
    inline TwoVector getAcceleration();
    inline double getMass();
    inline double getAngle();
    inline double getMaxAngle();
    inline double getLength();

    inline void setPosition(TwoVector);
    inline void setVelocity(TwoVector);
    inline void setAcceleration(TwoVector);
    inline void setAngle(double);

    virtual void move(double t);

private:
    BodyPart* fParent;
    double fMass;
    double fAngle;
    double fMaxAngle;
    TwoVector fPosition;
    TwoVector fVelocity;
    TwoVector fAcceleration;
    double fLength;
};

inline TwoVector BodyPart::getPosition(){return fPosition;}
inline TwoVector BodyPart::getVelocity(){return fVelocity;}
inline TwoVector BodyPart::getAcceleration(){return fAcceleration;}
inline double BodyPart::getMass(){return fMass;}
inline double BodyPart::getAngle(){return fAngle;}
inline double BodyPart::getMaxAngle(){return fMaxAngle;}
inline double BodyPart::getLength(){return fLength;}

inline void BodyPart::setPosition(TwoVector tV){fPosition=tV;}
inline void BodyPart::setVelocity(TwoVector tV){fVelocity=tV;}
inline void BodyPart::setAcceleration(TwoVector tV){fAcceleration=tV;}
inline void BodyPart::setAngle(double angle){fAngle=angle;}

#endif // BODYPART_H
