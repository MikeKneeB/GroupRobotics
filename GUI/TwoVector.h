/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   TwoVector.h
 * Author: hxw307
 *
 * Header for the TwoVector class, each TwoVector has
 * an x value and a y value.
 *
 * 15/11/2016
 */

#ifndef TWOVECTOR_H
#define TWOVECTOR_H

#include <math.h>

class TwoVector {
public:
    //Empty Constructor
    TwoVector();
    //Constructor using values
    TwoVector(double x, double y);
    //Constructor from array
    TwoVector(const double *);
    //Copy Constructor
    TwoVector(const TwoVector &);

    //Destructor
    virtual ~TwoVector();

    //Getters - declared inline
    inline double getX();
    inline double getY();
    //Setters - declared inline
    inline void setX(double);
    inline void setY(double);
    inline void setXY(double x,double y);
    /*
     * Function to return the dot product of this TwoVector
     * with another passed to it.
     */
    inline double dot(const TwoVector &) const;
    //Overides of the operators all declared inline
    inline void operator = (const TwoVector &);
    inline void operator += (const TwoVector &);
    inline void operator -= (const TwoVector &);
    inline void operator *= (double);
    inline TwoVector operator + (const TwoVector &);
    inline TwoVector operator - (const TwoVector &);
    inline TwoVector operator * (double);
    inline TwoVector operator / (double);
    //Function to return the absolute value of this vector
    inline double absolute();
    //Print function for debugging
    void print();

private:
    //Field variables
    double fX,fY;
};
//Getters
inline double TwoVector::getX(){return fX;}
inline double TwoVector::getY(){return fY;}
//Setters
inline void TwoVector::setX(double x){fX=x;}
inline void TwoVector::setY(double y){fY=y;}
inline void TwoVector::setXY(double x, double y){fX=x;fY=y;}
//Scalar Product
inline double TwoVector::dot(const TwoVector & tV) const {
    return fX*tV.fX + fY*tV.fY;
}
//Operators
inline void TwoVector::operator = (const TwoVector & tV){fX=tV.fX;fY=tV.fY;}
inline void TwoVector::operator += (const TwoVector & tV){fX+=tV.fX;fY+=tV.fY;}
inline void TwoVector::operator -= (const TwoVector & tV){fX-=tV.fX;fY-=tV.fY;}
inline void TwoVector::operator *= (double scale){fX*=scale;fY*=scale;}
inline TwoVector TwoVector::operator +(const TwoVector & tV){
    TwoVector vec;
    vec.fX=this->fX+tV.fX;
    vec.fY=this->fY+tV.fY;
    return vec;
}
inline TwoVector TwoVector::operator -(const TwoVector & tV){
    TwoVector vec;
    vec.fX=this->fX-tV.fX;
    vec.fY=this->fY-tV.fY;
    return vec;
}
inline TwoVector TwoVector::operator *(double scale){
    TwoVector vec;
    vec.fX=this->fX*scale;
    vec.fY=this->fY*scale;
    return vec;
}
inline TwoVector TwoVector::operator /(double scale){
    TwoVector vec;
    vec.fX=this->fX/scale;
    vec.fY=this->fY/scale;
    return vec;
}
inline double TwoVector::absolute(){
    return pow(pow(fX,2)+pow(fY,2),0.5);
}

#endif /* TWOVECTOR_H */

