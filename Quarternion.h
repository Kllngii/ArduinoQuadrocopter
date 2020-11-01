// Quaternion.h

#ifndef _QUATERNION_h
#define _QUATERNION_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
//#include "WProgram.h"
#endif

#endif

class Quaternion {
public:
	float r;
	float i;
	float j;
	float k;

	Quaternion();

	Quaternion axisAngle(float a, float b, float c);

	Quaternion(float nw, float nx, float ny, float nz);
	//Quaternion(float a, float b, float c);

	Quaternion multiply(Quaternion q);

	Quaternion invert();
	//String toString() {
	//	String tab = "	";
	//	String s = w;
	//	s += tab;
	//	s += x;
	//	s += tab;
	//	s += y;
	//	s += tab;
	//	s += z;
	//	s += tab;
	//	return s;
	//}

	float getMagnitude();
	Quaternion rotate(Quaternion q);
	void normalize();

	Quaternion getNormalized();
};
