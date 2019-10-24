#pragma once

#include <cnl/all.h>
#include <cnl/elastic_number.h>

using fixScalar = cnl::fixed_point<int64_t, -16>;

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

namespace util
{
	template <typename T>
	inline T ratio(T v, T a, T b);

    template<>
    inline fixScalar ratio(fixScalar v, fixScalar a, fixScalar b)
    {
        cnl::fixed_point<int64_t, -32> num;
        num = v - a;
        return num / (b - a);
    }
}

inline vec3fi operator-(const vec3fi& a, const vec3fi& b)
{
    vec3fi ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

inline vec3fi operator+(const vec3fi& a, const vec3fi& b)
{
    vec3fi ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}

inline vec3fi operator*(const vec3fi& a, fixScalar b)
{
    vec3fi ret;
    ret.x = a.x * b;
    ret.y = a.y * b;
    ret.z = a.z * b;
    return ret;
}

inline vec3fi operator*(fixScalar b, const vec3fi& a)
{
    return a*b;
}

inline vec3fi cross(const vec3fi& a, const vec3fi& b)
{
    vec3fi ret;
    ret.x = a.y * b.z - a.z * b.y;
    ret.y = a.z * b.x - a.x * b.z;
    ret.z = a.x * b.y - a.y * b.x;
    return ret;
}

inline fixScalar norm(const vec3fi& a)
{
    fixScalar sum = a.x * a.x;
    sum += a.y * a.y;
    sum += a.z * a.z;    
    return cnl::sqrt(static_cast<cnl::fixed_point<int32_t, -16>>(sum));
}

inline vec3fi normalize(const vec3fi& a)
{
    auto n = norm(a);
    if (n == 0)
    {
        return a;
    }
    auto invn = cnl::fixed_point<int64_t, -32>(1.0) / n;
    return a * invn;
}

inline fixScalar dot(const vec3fi& a, const vec3fi& b)
{
    fixScalar ret = a.x * b.x;
    ret += a.y * b.y;
    ret += a.z * b.z;
    return ret;
}

inline vec3fi rcp(const vec3fi& a)
{
    vec3fi ret;
    ret.x = cnl::fixed_point<int64_t, -32>(1.0) / a.x;
    ret.y = cnl::fixed_point<int64_t, -32>(1.0) / a.y;
    ret.z = cnl::fixed_point<int64_t, -32>(1.0) / a.z;
    return ret;
}