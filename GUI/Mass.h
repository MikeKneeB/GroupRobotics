#ifndef MASS_H
#define MASS_H

#include "TwoVector.h"

#include <string>
#include <fstream>
#include <algorithm>

class Mass
{
public:
    Mass(double mass, std::string filepath);
    virtual ~Mass();

    void readFile();
    TwoVector readElement(int index);
    void findLength();

    int inline getLength();

private:
    double fMass;
    std::string fFilepath;
    double* fT;
    double* fX;
    double* fY;
    int fLength;
};

int inline Mass::getLength(){return fLength;}

#endif // MASS_H
