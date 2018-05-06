#pragma once
#include "Eigen/Dense"

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

