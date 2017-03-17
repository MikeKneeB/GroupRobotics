
#include <math.h>
#include <iostream>

#include "ThreeVector.h"

ThreeVector::ThreeVector()
: fX(0.0), fY(0.0), fZ(0.0) {}

ThreeVector::ThreeVector(const ThreeVector & p)
: fX(p.fX), fY(p.fY), fZ(p.fZ) {}

ThreeVector::ThreeVector(double xx, double yy, double zz)
: fX(xx), fY(yy), fZ(zz) {}

ThreeVector::ThreeVector(const double * x0)
: fX(x0[0]), fY(x0[1]), fZ(x0[2]) {}

ThreeVector::~ThreeVector() {
}

double ThreeVector::Phi() const
{
   //return the  azimuth angle. returns phi from -pi to pi
   return fX == 0.0 && fY == 0.0 ? 0.0 : atan2(fY,fX);
}
//______________________________________________________________________________
double ThreeVector::Mag() const
{
   // return the magnitude (rho in spherical coordinate system)

   return sqrt(Mag2());
}

//______________________________________________________________________________
double ThreeVector::Perp() const
{
   //return the transverse component  (R in cylindrical coordinate system)

   return sqrt(Perp2());
}

//______________________________________________________________________________
double ThreeVector::Perp(const ThreeVector & p) const
{
   //return the transverse component (R in cylindrical coordinate system)

   return sqrt(Perp2(p));
}
//______________________________________________________________________________
double ThreeVector::Theta() const
{
   //return the polar angle
   return fX == 0.0 && fY == 0.0 && fZ == 0.0 ? 0.0 : atan2(Perp(),fZ);
}

//______________________________________________________________________________
ThreeVector ThreeVector::Unit() const
{
   // return unit vector parallel to this.
   double  tot2 = Mag2();
   double tot = (tot2 > 0) ?  1.0/sqrt(tot2) : 1.0;
   ThreeVector p(fX*tot,fY*tot,fZ*tot);
   return p;
}

//Implementation of the global operators; notice the absence of ThreeVector::
//They all return a newly constructed ThreeVector
ThreeVector operator + (const ThreeVector & a, const ThreeVector & b) {
   return ThreeVector(a.X() + b.X(), a.Y() + b.Y(), a.Z() + b.Z());
}

ThreeVector operator - (const ThreeVector & a, const ThreeVector & b) {
   return ThreeVector(a.X() - b.X(), a.Y() - b.Y(), a.Z() - b.Z());
}

ThreeVector operator * (const ThreeVector & p, double a) {
   return ThreeVector(a*p.X(), a*p.Y(), a*p.Z());
}

ThreeVector operator * (double a, const ThreeVector & p) {
   return ThreeVector(a*p.X(), a*p.Y(), a*p.Z());
}

void ThreeVector::Print()const
{
   //print vector parameters
   std::cout << "(x,y,z)=(" << X() << "," << Y() << "," << Z() << ")" << std::endl;
}


