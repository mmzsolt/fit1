#pragma once

#include "basic.hpp"
#include "basic_fixed.hpp"
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
	decltype(pos) v0 = pos + decltype(pos)(-w / 2.0f, -h / 2.0f, 0.0f);
	decltype(pos) v1 = pos + decltype(pos)(w / 2.0f, -h / 2.0f, 0.0f);
	decltype(pos) v2 = pos + decltype(pos)(-w / 2.0f, h / 2.0f, 0.0f);
	decltype(pos) v3 = pos + decltype(pos)(w / 2.0f, h / 2.0f, 0.0f);
	return{ Triangle{v0, v1, v2}, Triangle{v1, v3, v2} };	
}

class Triangle_fi
{
public:
	vec3fi m_v0;
	vec3fi m_v1;
	vec3fi m_v2;
	Intersection_fi intersect(Ray_fi r) const
	{
		// i'm too lazy, it's from here: https://github.com/marczych/RayTracer/blob/master/src/Triangle.cpp
		auto e1 = m_v1 - m_v0;
		auto e2 = m_v2 - m_v0;

		auto normal = cross(e1,e2);
		normal = normalize(normal);

		auto h = cross(r.m_dir,e2);
		auto a = dot(e1,h);

		if (a > -0.0001 && a < 0.0001)
			return Intersection_fi();

		auto f = 1 / a;
		auto s = r.m_pos - m_v0;

		auto u = f * (dot(s,h));

		if (u < 0.0 || u > 1.0)
			return Intersection_fi();

		auto q = cross(s,e1);
		auto v = f * (dot(r.m_dir,q));

		if (v < 0.0 || u + v > 1.0)
			return Intersection_fi();

		auto distance = f * dot(e2,q);

		// Ray Intersection
		if (distance > 0.0001) {
			auto point = r.m_pos + distance * r.m_dir;
			return Intersection_fi(point, distance);
		}

		return Intersection_fi();
	}

	std::string toString() const
	{
		return std::string() + "Triangle((" + util::to_string(m_v0) + "),(" + util::to_string(m_v1) + "),(" + util::to_string(m_v2) + "))";
	}

};

std::pair<Triangle_fi, Triangle_fi> makeQuad_fi(vec3fi pos, fixScalar w, fixScalar h)
{
	fixScalar zero(0);
	fixScalar two(2);
	decltype(pos) v0 = pos + decltype(pos)(zero - (w/two), zero - (h/two), zero);
	decltype(pos) v1 = pos + decltype(pos)(w / two, zero - (h / two), zero);
	decltype(pos) v2 = pos + decltype(pos)(zero - (w / two), h / two, zero);
	decltype(pos) v3 = pos + decltype(pos)(w / two, h / two, zero);
	return{ Triangle_fi{v0, v1, v2}, Triangle_fi{v1, v3, v2} };	
}
