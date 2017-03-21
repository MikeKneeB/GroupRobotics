
#ifndef THREEVECTOR_H_
#define THREEVECTOR_H_

//This class is the one used in the lectures, and is useful for this project

class ThreeVector {
public:
	//The default constructor.
	ThreeVector();
	//Another constructor.
	ThreeVector(double x, double y, double z);

	//Constructor from an array
	ThreeVector(const double *);

	//The copy constructor. The & indicates that the parameter is passed as reference,
	//so its content is not copied into another instance but the one passed at the invocation
	//is used directly
	ThreeVector(const ThreeVector &);

	//The destructor.
	virtual ~ThreeVector();

	//Declaration of access methods. inline instructs the compiler to replicate
	//the corresponding machine code each time they are invoked, rather than
	//jumping to the position of a single copy of machine code; this results in
	//faster execution, but larger executables. Declaring a method inline requires
	//the implementation to be done in the same file. const indicates that the object is not
	//modified by the execution of the method, thus the compiler can perform some
	//optimization
	//The components in cartesian coordinate system.
	inline double x()  const;
	inline double y()  const;
	inline double z()  const;
	inline double X()  const;
	inline double Y()  const;
	inline double Z()  const;
	inline double Px() const;
	inline double Py() const;
	inline double Pz() const;

	//inline but not const, since they are meant to modify the object
	//Set the components
	inline void SetX(double);
	inline void SetY(double);
	inline void SetZ(double);
	inline void SetXYZ(double x, double y, double z);

	//inline and const can be used wherever convenient and possible (const cannot be used
	//if something is modified during the execution) to optimize the code
	//Scalar product.
	inline double Dot(const ThreeVector &) const;

	//The transverse component (R in cylindrical coordinate system).
	double Perp() const;

	//The transverse component squared (R^2 in cylindrical coordinate system)
	inline double Perp2() const;

	//The transverse component w.r.t. given axis squared.
	inline double Perp2(const ThreeVector &) const;

	//The transverse component w.r.t. given axis.
	double Perp(const ThreeVector &) const;

	//The azimuth angle. returns phi from -pi to pi
	double Phi() const;

	//The polar angle.
	double Theta() const;

	//The magnitude squared (rho^2 in spherical coordinate system).
	inline double Mag2() const;

	//The magnitude (rho in spherical coordinate system).
	double Mag() const;

	//Declaration of operators acting on the invoking instance; they could be implemented
	//here or in the implementation file, if they were not declared inline
	//Assignment.
	inline ThreeVector & operator = (const ThreeVector &);

	//Addition.
	inline ThreeVector & operator += (const ThreeVector &);

	//Subtraction.
	inline ThreeVector & operator -= (const ThreeVector &);

	//Unary minus.
	inline ThreeVector operator - () const;

	//Scaling with real numbers.
	inline ThreeVector & operator *= (double);

	//Unit vector parallel to this.
	ThreeVector Unit() const;

	//Printing the content in a convenient way
	void Print() const;

private:

	double fX, fY, fZ;
	// The components.
};

//Declaration of operators without an invoking instance. They must be global, thus
//declared outside the scope of the class
//Addition of 3-vectors.
ThreeVector operator + (const ThreeVector &, const ThreeVector &);

//Subtraction of 3-vectors.
ThreeVector operator - (const ThreeVector &, const ThreeVector &);

//Scaling of 3-vectors with a real number
ThreeVector operator * (const ThreeVector &, double a);
ThreeVector operator * (double a, const ThreeVector &);

//Implementation of all methods and operators declared inline
inline double ThreeVector::x()  const { return fX; }
inline double ThreeVector::y()  const { return fY; }
inline double ThreeVector::z()  const { return fZ; }
inline double ThreeVector::X()  const { return fX; }
inline double ThreeVector::Y()  const { return fY; }
inline double ThreeVector::Z()  const { return fZ; }
inline double ThreeVector::Px() const { return fX; }
inline double ThreeVector::Py() const { return fY; }
inline double ThreeVector::Pz() const { return fZ; }

inline void ThreeVector::SetX(double xx) { fX = xx; }
inline void ThreeVector::SetY(double yy) { fY = yy; }
inline void ThreeVector::SetZ(double zz) { fZ = zz; }

inline void ThreeVector::SetXYZ(double xx, double yy, double zz) {
   fX = xx;
   fY = yy;
   fZ = zz;
}

inline double ThreeVector::Perp2() const { return fX*fX + fY*fY; }

inline double ThreeVector::Mag2() const { return fX*fX + fY*fY + fZ*fZ; }

inline double ThreeVector::Dot(const ThreeVector & p) const {
   return fX*p.fX + fY*p.fY + fZ*p.fZ;
}

inline double ThreeVector::Perp2(const ThreeVector & p)  const {
   double tot = p.Mag2();
   double ss  = Dot(p);
   double per = Mag2();
   if (tot > 0.0) per -= ss*ss/tot;
   if (per < 0)   per = 0;
   return per;
}

//All operators involving assignment return the invoking instance itself
//by dereferencing the pointer this
inline ThreeVector & ThreeVector::operator = (const ThreeVector & p) {
   fX = p.fX;
   fY = p.fY;
   fZ = p.fZ;
   return *this;
}

inline ThreeVector& ThreeVector::operator += (const ThreeVector & p) {
   fX += p.fX;
   fY += p.fY;
   fZ += p.fZ;
   return *this;
}

inline ThreeVector& ThreeVector::operator -= (const ThreeVector & p) {
   fX -= p.fX;
   fY -= p.fY;
   fZ -= p.fZ;
   return *this;
}

inline ThreeVector ThreeVector::operator - () const {
   return ThreeVector(-fX, -fY, -fZ);
}

inline ThreeVector& ThreeVector::operator *= (double a) {
   fX *= a;
   fY *= a;
   fZ *= a;
   return *this;
}

#endif /* THREEVECTOR_H_ */
