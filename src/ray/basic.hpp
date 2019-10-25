#pragma once
#include "Eigen/Dense"
#include <float.h>

class Ray
{
public:
	Eigen::Vector3f m_pos;
	Eigen::Vector3f m_dir;  // always normalized
};

class Intersection
{
public:
	Intersection() {}
	Intersection(Eigen::Vector3f p, float f)
		: m_pos(p)
		, m_depth(f)
	{}
	Eigen::Vector3f m_pos;
	float m_depth = FLT_MAX;
	bool isValid() const { return m_depth < FLT_MAX; }
};

class Primitive
{
public:
	Primitive() {}
	virtual ~Primitive() {}
	virtual Intersection intersect(Ray r) const = 0;
	virtual std::string toString() const = 0;
	Eigen::Vector3f m_color = {1.0f, 1.0f, 1.0f};
};