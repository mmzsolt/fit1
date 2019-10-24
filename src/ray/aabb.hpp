#pragma once

#include "basic.hpp"
#include "basic_fixed.hpp"
#include "util.hpp"

class AABB
{
public:
	Eigen::Vector3f m_min;
    Eigen::Vector3f m_max;
    Intersection intersect(Ray r) const
	{
        Eigen::Vector3f rcpRayDir = r.m_dir.array().inverse();
        Eigen::Vector3f t0 = (m_min - r.m_pos).array() * rcpRayDir.array();
        Eigen::Vector3f t1 = (m_max - r.m_pos).array() * rcpRayDir.array();
        Eigen::Vector3f tmin = util::min(t0, t1);
        Eigen::Vector3f tmax = util::max(t0, t1);
        float tminc = util::max_component(tmin);
        float tmaxc = util::min_component(tmax);
        if (tminc > tmaxc)
        {
            return Intersection();
        }
        else
        {
       		auto point = r.m_pos + (r.m_dir * tminc);
            return Intersection(point, tminc);
        }        
    }
    void fix()
    {
        if (m_min.norm() > m_max.norm())
        {
            std::swap(m_min, m_max);
        }
    }
	std::string toString() const
	{
		return std::string() + "AABB((" + util::to_string(m_min) + "),(" + util::to_string(m_max) + "))";
	}
};