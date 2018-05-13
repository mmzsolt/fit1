#pragma once

#include "basic.hpp"
#include "util.hpp"

class Sphere
{
public:
	Eigen::Vector3f m_pos;
	float m_radius;
	Intersection intersect(Ray r) const
	{
		// i'm too lazy, it's from here: https://github.com/marczych/RayTracer/blob/master/src/Sphere.cpp
		auto deltap = r.m_pos - m_pos;
		float a = 1.0f;
		float b = deltap.dot(r.m_dir) * 2.0f;
		float c = deltap.dot(deltap) - (m_radius * m_radius);

		float disc = b * b - 4 * a * c;
		if (disc < 0) {
			return Intersection(); // No intersection.
		}

		disc = sqrt(disc);

		float q;
		if (b < 0) {
			q = (-b - disc) * 0.5f;
		}
		else {
			q = (-b + disc) * 0.5f;
		}

		float r1 = q / a;
		float r2 = c / q;

		if (r1 > r2) {
			float tmp = r1;
			r1 = r2;
			r2 = tmp;
		}

		float distance = r1;
		if (distance < 0) {
			distance = r2;
		}

		if (distance < 0 || isnan(distance)) {
			return Intersection(); // No intersection.
		}

		auto point = r.m_pos + (r.m_dir * distance);

		return Intersection( point, distance );
	}
	std::string toString() const
	{
		return std::string() + "Sphere((" + util::to_string(m_pos) + ")," + std::to_string(m_radius) + ")";
	}
};