#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir, const double tm = 0.0 ) : origin(o), direction(dir) {
		id = nextId++;
		time = tm;
	};

	Vector origin;
	Vector direction;

	int id;
	static int nextId;

	double time;
};
#endif