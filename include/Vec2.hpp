#ifndef VEC
#define VEC
#include <math.h>
struct Vec2{
	float x,y;

	float dist() const{
		return sqrt(x*x+y*y);
	}

	
};
#endif