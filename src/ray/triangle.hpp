#pragma once

#include "basic.hpp"
#include "util.hpp"

class Triangle
{
public:
	Eigen::Vector3f m_v0;
	Eigen::Vector3f m_v1;
	Eigen::Vector3f m_v2;
	Intersection intersect(Ray r) const
	{
		// i'm too lazy, it's from here: https://github.com/marczych/RayTracer/blob/master/src/Triangle.cpp
		auto e1 = m_v1 - m_v0;
		auto e2 = m_v2 - m_v0;

		auto normal = e1.cross(e2);
		normal.normalize();

		auto h = r.m_dir.cross(e2);
		auto a = e1.dot(h);

		if (a > -0.00001 && a < 0.00001)
			return Intersection();

		auto f = 1 / a;
		auto s = r.m_pos - m_v0;

		auto u = f * s.dot(h);

		if (u < 0.0 || u > 1.0)
			return Intersection();

		auto q = s.cross(e1);
		auto v = f * r.m_dir.dot(q);

		if (v < 0.0 || u + v > 1.0)
			return Intersection();

		auto distance = f * e2.dot(q);

		// Ray Intersection
		if (distance > 0.00001) {
			auto point = r.m_pos + distance * r.m_dir;
			return Intersection(point, distance);
		}

		return Intersection();
	}

	std::string toString() const
	{
		return std::string() + "Triangle((" + util::to_string(m_v0) + "),(" + util::to_string(m_v1) + "),(" + util::to_string(m_v2) + "))";
	}

};

std::pair<Triangle, Triangle> makeQuad(Eigen::Vector3f pos, float w, float h)
{
	//auto v0 = pos + decltype(pos)(-w / 2.0f, -h / 2.0f, 0.0f);
	auto v0 = pos + Eigen::Vector3f(-w / 2.0f, -h / 2.0f, 0.0f);
	auto v1 = pos + decltype(pos)(w / 2.0f, -h / 2.0f, 0.0f);
	auto v2 = pos + decltype(pos)(-w / 2.0f, h / 2.0f, 0.0f);
	auto v3 = pos + decltype(pos)(w / 2.0f, h / 2.0f, 0.0f);
	return{ Triangle{v0, v1, v2}, Triangle{v1, v3, v2} };
}