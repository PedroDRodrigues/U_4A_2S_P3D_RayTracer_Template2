#ifndef RAY_H
#define RAY_H

#include "vector.h"

class Ray
{
public:
	Ray(const Vector& o, const Vector& dir ) : origin(o), direction(dir) {
		id = nextId++;
	};

	Vector origin;
	Vector direction;

	int id;
	int nextId = 0;
};
#endif