#include "Mass.h"
#include <iostream>

Mass::Mass(double mass, std::string filepath):
    fMass(mass),
    fFilepath(filepath)
{
    findLength();
    readFile();
}

Mass::~Mass(){
    delete(fT);
    delete(fX);
    delete(fY);
}

void Mass::readFile(){
    std::ifstream inFile(fFilepath);

    fT=new double[fLength];
    fX=new double[fLength];
    fY=new double[fLength];

    double t;
    double x;
    double y;

    int i=0;
    while(inFile>>t>>x>>y){
        fT[i]=t;
        fX[i]=x;
        fY[i]=y;
        i++;
    }
}

void Mass::findLength(){
    std::ifstream inFile(fFilepath);

    int length=std::count(std::istreambuf_iterator<char>(inFile),
                 std::istreambuf_iterator<char>(), '\n');
    fLength=length;
}

TwoVector Mass::readElement(int index){
    return TwoVector(fX[index],fY[index]);
}
