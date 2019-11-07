#pragma once

#include "util.hpp"
#include "float.h"

// thanks to Christer Ericson
inline float ClosestPtSegmentSegment(const Eigen::Vector3f& p1, const Eigen::Vector3f& q1, const Eigen::Vector3f& p2, const Eigen::Vector3f& q2,
                              float &s, float &t, Eigen::Vector3f &c1, Eigen::Vector3f &c2)
{
    auto d1 = q1 - p1; // Direction vector of segment S1
    auto d2 = q2 - p2; // Direction vector of segment S2
    auto r = p1 - p2;
    float a = d1.dot(d1); //Dot(d1, d1); // Squared length of segment S1, always nonnegative
    float e = d2.dot(d2); //Dot(d2, d2); // Squared length of segment S2, always nonnegative
    float f = d2.dot(r);  //Dot(d2, r);

    // Check if either or both segments degenerate into points
    if (util::isNegative(a) && util::isNegative(e)) {
        // Both segments degenerate into points
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return (c1-c2).dot(c1-c2); //Dot(c1 - c2, c1 - c2);
    }
    if (util::isNegative(a)) {
        // First segment degenerates into a point
        s = 0.0f;
        t = f / e; // s = 0 => t = (b*s + f) / e = f / e
        t = util::clamp(t, 0.0f, 1.0f);
    } else {
        float c = d1.dot(r); //Dot(d1, r);
        if (util::isNegative(e)) {
            // Second segment degenerates into a point
            t = 0.0f;
            s = util::clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
        } else {
            // The general nondegenerate case starts here
            float b = d1.dot(d2); //Dot(d1, d2);
            float denom = a*e-b*b; // Always nonnegative

            // If segments not parallel, compute closest point on L1 to L2, and
            // clamp to segment S1. Else pick arbitrary s (here 0)
            if (denom != 0.0f) {
                s = util::clamp((b*f - c*e) / denom, 0.0f, 1.0f);
            } else s = 0.0f;

            // Compute point on L2 closest to S1(s) using
            // t = Dot((P1+D1*s)-P2,D2) / Dot(D2,D2) = (b*s + f) / e
            t = (b*s + f) / e;

            // If t in [0,1] done. Else clamp t, recompute s for the new value
            // of t using s = Dot((P2+D2*t)-P1,D1) / Dot(D1,D1)= (t*b - c) / a
            // and clamp s to [0, 1]
            if (util::isNegative(t)) {
                t = 0.0f;
                s = util::clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = util::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    return sqrt((c1-c2).dot(c1-c2)); //Dot(c1 - c2, c1 - c2);
}

inline void ClosestPtPointSegment(const Eigen::Vector3f& c, const Eigen::Vector3f& a, const Eigen::Vector3f& b,
float &t, Eigen::Vector3f& d)
{
    const auto ab = b - a;
    t = (c - a).dot(ab);
    if (util::isNegative(t))
    {
        t = 0.0f;
        d = a;
    }
    else
    {
        auto denom = ab.dot(ab);
        if (t >= denom)
        {
            t = 1.0f;
            d = b;
        }
        else
        {
            t = t / denom;
            d = a + t * ab;
        }
    }    
}

inline void ClosestPtPointAABB(const Eigen::Vector3f& p, const Eigen::Vector3f& mi, const Eigen::Vector3f& ma,
float& dist, Eigen::Vector3f& r)
{
    r = util::max(p, mi);
    r = util::min(r, ma);
    dist = util::distance(r, p);
}

inline bool intersect(const Capsule& capsule, const AABB& aabb, Eigen::Vector3f& pC, Eigen::Vector3f& pS, float& dist)
{
    float s, t;
    return false;
}

std::array<Eigen::Vector3f, 8> AABBtoVertices(const AABB& aabb)
{
    std::array<Eigen::Vector3f, 8> ret;
    ret[0] = aabb.m_min;
    ret[1] = {aabb.m_max.x(), aabb.m_min.y(), aabb.m_min.z()};
    ret[2] = {aabb.m_max.x(), aabb.m_max.y(), aabb.m_min.z()};
    ret[3] = {aabb.m_min.x(), aabb.m_max.y(), aabb.m_min.z()};
    ret[4] = {aabb.m_min.x(), aabb.m_min.y(), aabb.m_max.z()};
    ret[5] = {aabb.m_max.x(), aabb.m_min.y(), aabb.m_max.z()};
    ret[6] = aabb.m_max;
    ret[7] = {aabb.m_min.x(), aabb.m_max.y(), aabb.m_max.z()};
    return ret;
}

inline std::vector<Eigen::Vector3f> intersectTest(const Capsule& capsule, const AABB& aabb)
{
    std::vector<Eigen::Vector3f> pts(4);
    float s, t;
    auto vertices = AABBtoVertices(aabb);
    std::array<std::pair<float, Eigen::Vector3f>, 8> distances;
    auto distIt = std::begin(distances);
    for( auto vert: vertices )
    {
        Eigen::Vector3f temp;
        ClosestPtPointSegment(vert, capsule.m_min, capsule.m_max, t, temp);
        *distIt++ = {util::distance(vert, temp), vert};
    }
    std::sort(std::begin(distances), std::end(distances), [](const auto& d1, const auto& d2)
        {return d1.first < d2.first;});

    // these 4 are the closest points on the AABB to the capsule, they should lie on the same face
    for(int i = 0; i < 4; ++i)
    {
        pts[i] = distances[i].second;
    }    

    return pts;
}


inline bool intersect(const Sphere& sphere1, const Sphere& sphere2, Eigen::Vector3f& pS1, Eigen::Vector3f& pS2, float& dist)
{
    auto [vecLen, vec] = util::directionAndDistance(sphere1.m_pos, sphere2.m_pos);
    pS1 = sphere1.m_pos + vec * sphere1.m_radius;
    pS2 = sphere2.m_pos - vec * sphere2.m_radius;
    dist = vecLen - sphere1.m_radius - sphere2.m_radius;
    return util::isNegative(dist);
}

inline bool intersect(const Capsule& capsule, const Sphere& sphere, Eigen::Vector3f& pC, Eigen::Vector3f& pS, float& dist)
{
    float t;
    Eigen::Vector3f segPt;
    // calculate the closest point on the segment to the center of the sphere
    ClosestPtPointSegment(sphere.m_pos, capsule.m_min, capsule.m_max, t, segPt);
    // connecting vector between intersection point on segment and sphere center
    auto [vecLen, vec] = util::directionAndDistance(sphere.m_pos, segPt);
    pS = sphere.m_pos + vec * sphere.m_radius;
    pC = segPt - vec * capsule.m_radius;
    dist = vecLen - sphere.m_radius - capsule.m_radius;
    return util::isNegative(dist);
}

inline bool intersect(const Capsule& capsule1, const Capsule& capsule2, Eigen::Vector3f& pC1, Eigen::Vector3f& pC2, float& dist)
{
    float s, t;
    Eigen::Vector3f seg1P, seg2P;
    ClosestPtSegmentSegment(capsule1.m_min, capsule1.m_max, capsule2.m_min, capsule2.m_max, s, t, seg1P, seg2P);
    auto [segDist, segVec] = util::directionAndDistance(seg1P, seg2P);
    dist = segDist - capsule1.m_radius - capsule2.m_radius;
    pC1 = seg1P + segVec * capsule1.m_radius;
    pC2 = seg2P - segVec * capsule2.m_radius;
    return util::isNegative(dist);
}

inline bool intersect(const AABB& aabb, const Sphere& sphere, Eigen::Vector3f& pA, Eigen::Vector3f& pS, float& dist)
{
    ClosestPtPointAABB(sphere.m_pos, aabb.m_min, aabb.m_max, dist, pA);
    dist -= sphere.m_radius;
    auto vec = util::direction(sphere.m_pos, pA);
    pS = sphere.m_pos + vec * sphere.m_radius;
    return util::isNegative(dist);
}