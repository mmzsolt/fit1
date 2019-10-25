#pragma once

#include "basic.hpp"
#include "basic_fixed.hpp"
#include "util.hpp"

class Sphere : public Primitive
{
public:
	Sphere() : Primitive() {}
	Sphere(Eigen::Vector3f _pos, float _radius) : Primitive(), m_pos(_pos ), m_radius(_radius) {}
	Eigen::Vector3f m_pos;
	float m_radius;
	virtual Intersection intersect(Ray r) const override
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
	virtual std::string toString() const override
	{
		return std::string() + "Sphere((" + util::to_string(m_pos) + ")," + std::to_string(m_radius) + ")";
	}
};

class Sphere_fi
{
public:
	vec3fi m_pos;
	fixScalar m_radius;
	Intersection_fi intersect(Ray_fi r) const
	{
		// i'm too lazy, it's from here: https://github.com/marczych/RayTracer/blob/master/src/Sphere.cpp
		const vec3fi deltap = r.m_pos - m_pos;
		fixScalar b = dot(deltap,r.m_dir) * 2;
		fixScalar c = dot(deltap, deltap) - (m_radius * m_radius);

		fixScalar disc = (b * b) - (4 * c);
		if (disc < 0) {
			return Intersection_fi(); // No intersection.
		}

		disc = sqrt(static_cast<cnl::fixed_point<int32_t, -16>>(disc));

		fixScalar q;
		const cnl::fixed_point<int64_t, -1> half(0.5);
		if (b < 0) {
			q = (-b - disc) * half;
		}
		else {
			q = (-b + disc) * half;
		}

		auto r2 = static_cast<cnl::fixed_point<int64_t,-32>>(c) / q;
		decltype(r2) r1 = q;

		if (r1 > r2) {
			std::swap(r1, r2);
		}

		auto distance = r1;
		if (distance < 0) {
			distance = r2;
		}

		if (distance < 0 || q == 0) {
			return Intersection_fi(); // No intersection.
		}

		auto point = r.m_pos + (r.m_dir * distance);

		return Intersection_fi( point, distance );
	}
	std::string toString() const
	{
		return std::string() + "Sphere((" + util::to_string(m_pos) + ")," + std::to_string(static_cast<float>(m_radius)) + ")";
	}
};