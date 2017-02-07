/*
 * File:   TwoVector.cpp
 * Author: hxw307
 *
 * The class file for the TwoVector class. This file includes
 * all functions that were not declared inline in the header file.
 *
 * 15/11/2016
 */

#include <iostream>
#include "TwoVector.h"

TwoVector::TwoVector() {
    fX=0.0;
    fY=0.0;
}
TwoVector::TwoVector(double x, double y){
    fX=x;
    fY=y;
}
TwoVector::TwoVector(const double * tV){
    fX=tV[0];
    fY=tV[1];
}
TwoVector::TwoVector(const TwoVector & tV){
    fX=tV.fX;
    fY=tV.fY;
}

TwoVector::~TwoVector(){}

void TwoVector::print(){
    //Print x and y
    std::cout << "(x,y)=(" << TwoVector::getX() << "," << TwoVector::getY() << ")" << std::endl;
}
