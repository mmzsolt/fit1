#pragma once

#include "basic.hpp"
#include "basic_fixed.hpp"
#include "util.hpp"

class Capsule : public Primitive
{
public:
    Capsule() : Primitive() {}
    Capsule(Eigen::Vector3f _min, Eigen::Vector3f _max, float _r) : Primitive(), m_min(_min), m_max(_max), m_radius(_r) {}
	Eigen::Vector3f m_min;
    Eigen::Vector3f m_max;
    float m_radius;
    virtual Intersection intersect(Ray r) const override
	{
        // i was lazy, it's based on this: http://www.iquilezles.org/www/articles/intersectors/intersectors.htm
        auto ba = m_max - m_min;
        auto oa = r.m_pos - m_min;
        auto baba = ba.dot(ba);
        auto bard = ba.dot(r.m_dir);
        auto baoa = ba.dot(oa);
        auto rdoa = r.m_dir.dot(oa);
        auto oaoa = oa.dot(oa);
        auto a = baba - bard * bard;
        auto b = baba * rdoa - baoa * bard;
        auto c = baba * oaoa - baoa * baoa - m_radius * m_radius * baba;
        auto h = b*b - a*c;
        if (h >= 0.0)
        {
            auto t = (-b - sqrt(h)) / a;
            auto y = baoa + t * bard;
            // body
            if ( y > 0.0 && y < baba )
            {
           		auto point = r.m_pos + (r.m_dir * t);
                // need to think about this a bit more
                //auto centerp = m_min + sqrt(y) * ba;
                //Eigen::Vector3f normal = point - centerp;
                //normal.normalize();
                auto norm2 = normalAt(point);
                return Intersection(point, t, norm2);
            }
            // caps
            auto oc = y < 0.0 ? oa : r.m_pos - m_max;
            b = r.m_dir.dot(oc);
            c = oc.dot(oc) - m_radius * m_radius;
            h = b * b - c;
            if ( h > 0.0 )
            {
                t = -b - sqrt(h);
           		auto point = r.m_pos + (r.m_dir * t);
                Eigen::Vector3f normal = y < 0.0 ? point - m_min : point - m_max;
                normal.normalize();
                // this is the same, but here we get the result faster, maybe :)
                //auto norm2 = normalAt(point);
                return Intersection(point, t, normal);
            }
        }
        return Intersection();
    }
    Eigen::Vector3f normalAt(const Eigen::Vector3f pos) const
    {
        auto ba = m_max - m_min;
        auto pa = pos - m_min;
        auto h = util::clamp(pa.dot(ba) / ba.dot(ba), 0.0f, 1.0f);
        auto ret = (pa - h * ba)/m_radius;
        return ret;
    }
	virtual std::string toString() const override
	{
		return std::string() + "Capsule((" + util::to_string(m_min) + "),(" + util::to_string(m_max) + ")," + std::to_string(m_radius)+")";
	}
};