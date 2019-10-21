#pragma once

#include <cnl/all.h>
#include <cnl/elastic_number.h>

using fixScalar = cnl::fixed_point<int64_t, 31, 32>;

struct vec3fi
{
    vec3fi() : x(0), y(0), z(0) {}
    vec3fi(fixScalar _x, fixScalar _y, fixScalar _z) : x(_x), y(_y), z(_z) {}
    fixScalar x;
    fixScalar y;
    fixScalar z;
};

class Ray_fi
{
public:
    vec3fi m_pos;
    vec3fi m_dir;
};

class Intersection_fi
{
public:
    Intersection_fi() {}
	Intersection_fi(vec3fi p, fixScalar f)
		: m_pos(p)
		, m_depth(f)
	{}
	vec3fi m_pos;
	fixScalar m_depth = cnl::numeric_limits<fixScalar>::max();
	bool isValid() const { return m_depth < cnl::numeric_limits<fixScalar>::max(); }
};

vec3fi operator-(const vec3fi& a, const vec3fi& b);

vec3fi operator+(const vec3fi& a, const vec3fi& b);