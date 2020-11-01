/*
 Name:    Quarternion.cpp
 Created: 17.08.2019 12:42:39
 Author:  Till Koch, Linus Lingstaedt, Lasse Kelling
*/

#include "Quarternion.h"

Quaternion::Quaternion() {
	r = 1.0f;
	i = 0.0f;
	j = 0.0f;
	k = 0.0f;
}

Quaternion::Quaternion(float nw, float nx, float ny, float nz) {
	r = nw;
	i = nx;
	j = ny;
	k = nz;
}
//Quaternion::Quaternion(float a, float b, float c) {
//}
Quaternion Quaternion::axisAngle(float a, float b, float c) {
	float d = sqrt(a*a + b * b + c * c);
	if (d == 0) {
		return Quaternion(1, 0, 0, 0);
	}
	else {
		float s = sin(d / 2) / d;
		return Quaternion(cos(d / 2), a*s, b*s, c*s);
	}
}

Quaternion Quaternion::multiply(Quaternion q) {
	// Quaternion multiplication is defined by:
	//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
	//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
	//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
	//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
	return Quaternion(
		r*q.r - i * q.i - j * q.j - k * q.k,  // new w
		r*q.i + i * q.r + j * q.k - k * q.j,  // new x
		r*q.j - i * q.k + j * q.r + k * q.i,  // new y
		r*q.k + i * q.j - j * q.i + k * q.r); // new z
}

Quaternion Quaternion::invert() {
	return Quaternion(r, -i, -j, -k);
}
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

float Quaternion::getMagnitude() {
	return sqrt(r*r + i * i + j * j + k * k);
}
Quaternion Quaternion::rotate(Quaternion q) {
	Quaternion qq = Quaternion(r, i, j, k);
	qq = q.multiply(qq);
	q = q.invert();
	qq = qq.multiply(q);
	return qq;

}
void Quaternion::normalize() {
	float m = getMagnitude();
	r /= m;
	i /= m;
	j /= m;
	k /= m;
}

Quaternion Quaternion::getNormalized() {
	Quaternion qu(r, i, j, k);
	qu.normalize();
	return qu;
}
