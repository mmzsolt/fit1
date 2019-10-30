#pragma once
#include "Eigen/Dense"
#include <float.h>

class Ray
{
public:
	Eigen::Vector3f m_pos;
	Eigen::Vector3f m_dir;  // always normalized
	Ray(decltype(m_pos)_pos, decltype(m_dir)_dir) : m_pos(_pos), m_dir(_dir) {}
};

class Primitive;

class Intersection
{
public:
	Intersection() {}
	Intersection(Eigen::Vector3f p, float f)
		: m_pos(p)
		, m_depth(f)
	{}
	Intersection(Eigen::Vector3f p, float f, Eigen::Vector3f n, const Primitive* prim)
		: m_pos(p)
		, m_depth(f)
		, m_normal(n)
		, m_primitive(prim)
	{}
	Eigen::Vector3f m_pos;
	float m_depth = FLT_MAX;
	const Primitive* m_primitive = nullptr;
	Eigen::Vector3f m_normal = {0.0f, 0.0f, 0.0f};
	bool isValid() const { return m_depth < FLT_MAX; }	
};

class Primitive
{
public:
	Primitive() {}
	virtual ~Primitive() {}
	virtual Intersection intersect(Ray r) const = 0;
	virtual std::string toString() const = 0;
};