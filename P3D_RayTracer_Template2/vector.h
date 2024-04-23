#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>
#include <cmath>
#include <cfloat>
// FUI EU QUE ADICIONEI O RANDOM
#include <random>

using namespace std;

class Vector
{
public:
	Vector(){};
	Vector(float x, float y, float z);
	Vector(const Vector& v);

	float length();

	float getAxisValue(int axis);

	Vector&	normalize();
	Vector operator=(const Vector& v);
	Vector operator+( const Vector& v );
	Vector operator-( const Vector& v );
	Vector operator*( float f );
	float  operator*(const Vector& v);   //inner product
	Vector operator/( float f );
	Vector operator%( const Vector& v); //external product
	Vector&	operator-=	(const Vector& v);
	Vector&	operator-=	(const float v);
	Vector&	operator*=	(const float v);
	Vector&	operator+=	(const float v);
	
	//static float dot(const Vector& a, const Vector& b);  // Dot product
	static Vector cross(const Vector& a, const Vector& b);  // Cross product
	static Vector random_in_unit_sphere() {
		static std::mt19937 generator(std::random_device{}());
		static std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

		Vector point;
		do {
			point = Vector(distribution(generator), distribution(generator), distribution(generator)) * 2.0f - Vector(1.0f, 1.0f, 1.0f);
		} while (point.length_squared() >= 1.0f);
		return point;
	}

	float length_squared() const {
		return x * x + y * y + z * z;
	}

	/*static Vector random_in_unit_disk() {
		static std::mt19937 generator(std::random_device{}());
		static std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

		Vector point;
		do {
			point = 2.0f * Vector(distribution(generator), distribution(generator), 0.0f) - Vector(1.0f, 1.0f, 0.0f);
		} while (point.length_squared() >= 1.0f);
		return point;
	}*/ // FOI O PEDRO QUE CRIOU ISTO, ACHO QUE VAI SER PRECISO NO FUTURO
	float x;
	float y;
	float z;

     friend inline
  istream&	operator >>	(istream& s, Vector& v)
	{ return s >> v.x >> v.y >> v.z; }
  
};

#endif