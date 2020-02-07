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

// from iq :)
inline float boxDistance(const Eigen::Vector3f& p, const Eigen::Vector3f& rad)
{
    auto d = util::abs(p) - rad;
    Eigen::Vector3f zero(0.0f, 0.0f, 0.0f);
    return util::max(d, zero).lpNorm<2>() + std::min(util::max_component(d), 0.0f);
}

inline void ClosestPtPointAABB(const Eigen::Vector3f& p, const Eigen::Vector3f& mi, const Eigen::Vector3f& ma,
float& dist, Eigen::Vector3f& r)
{
    r = util::max(p, mi);
    r = util::min(r, ma);
    //dist = util::distance(r, p);
    //want signed distance
    auto center = util::midpoint(mi, ma);
    auto semisize = center-mi;
    // yeah, i am this sure in my maths :(
    assert(semisize.x() > 0.0f && semisize.y() > 0.0f && semisize.y() > 0.0f);
    dist = boxDistance(p - center, semisize);
}

inline void ClosestPtSegmentPlane(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& pn,
float pd, float& t, Eigen::Vector3f& q)
{
    // Compute the t value for the directed line ab intersecting the plane
    auto ab = b - a;
    t = (pd - pn.dot(a)) / pn.dot(ab);
    q = a + t * ab;
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
    struct dist
    {
        Eigen::Vector3f m_aabbCorner;
        Eigen::Vector3f m_capsulePoint;
        float m_dist;
    };
    std::vector<Eigen::Vector3f> pts(4);
    float s, t;
    auto vertices = AABBtoVertices(aabb);
    std::array<dist, 8> distances;
    auto distIt = std::begin(distances);
    for( auto vert: vertices )
    {
        Eigen::Vector3f temp;
        ClosestPtPointSegment(vert, capsule.m_min, capsule.m_max, t, temp);
        *distIt++ = {vert, temp, util::distance(vert, temp)};
    }
    std::sort(std::begin(distances), std::end(distances), [](const auto& d1, const auto& d2)
        {return d1.m_dist < d2.m_dist;});

    // these 4 are the closest points on the AABB to the capsule, they should lie on the same face
    for(int i = 0; i < 4; ++i)
    {
        pts[i] = distances[i].m_aabbCorner;
    }

    auto planeNormal = util::calcNormal(distances[0].m_aabbCorner, distances[1].m_aabbCorner, distances[2].m_aabbCorner);
    float planeD = planeNormal.dot(distances[0].m_aabbCorner);
    Eigen::Vector3f planeIntersection;
    ClosestPtSegmentPlane(capsule.m_min, capsule.m_max, planeNormal, planeD, t, planeIntersection);

    // if t is between 0 and 1 then the segment intersects the plane
    if (t >= 0.0f && t <= 1.0f)
    {

    }

    pts.push_back(planeIntersection);

    return pts;
}

inline Eigen::Vector3f Corner(const AABB& b, int n)
{
    Eigen::Vector3f p;
    p.x() = ((n & 1) ? b.m_max.x() : b.m_min.x());
    p.y() = ((n & 2) ? b.m_max.y() : b.m_min.y());
    p.z() = ((n & 4) ? b.m_max.z() : b.m_min.z());
    return p;
}

// not really found in Christer's book, let's try to implement it
inline int IntersectSegmentCapsule(const Eigen::Vector3f& sa, const Eigen::Vector3f& sb, const Eigen::Vector3f& ca, const Eigen::Vector3f& cb, float r, float& t)
{
    //temporaries
    float tt;
    Eigen::Vector3f c1, c2;
    ClosestPtSegmentSegment(sa, sb, ca, cb, t, tt, c1, c2);
    return (util::distance(c1, c2) < r) ? 1 : 0;
}

// Intersect ray R(t) = p + t*d against AABB a. When intersecting,
// return intersection distance tmin and point q of intersection
inline int IntersectRayAABB(const Eigen::Vector3f& p, const Eigen::Vector3f& d, const AABB& a, float &tmin, Eigen::Vector3f &q)
{
    tmin = 0.0f;          // set to -FLT_MAX to get first hit on line
    float tmax = FLT_MAX; // set to max distance ray can travel (for segment)

    // For all three slabs
    for (int i = 0; i < 3; i++) {
        if (std::abs(d[i]) < FLT_EPSILON) {
            // Ray is parallel to slab. No hit if origin not within slab
            if (p[i] < a.m_min[i] || p[i] > a.m_max[i]) return 0;
        } else {
            // Compute intersection t value of ray with near and far plane of slab
            float ood = 1.0f / d[i];
            float t1 = (a.m_min[i] - p[i]) * ood;
            float t2 = (a.m_max[i] - p[i]) * ood;
            // Make t1 be intersection with near plane, t2 with far plane
            if (t1 > t2) std::swap(t1, t2);
            // Compute the intersection of slab intersections intervals
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            // Exit with no collision as soon as slab intersection becomes empty
            if (tmin > tmax) return 0;
        }
    }
    // Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin) 
    q = p + d * tmin;
    return 1;
}


inline int IntersectMovingSphereAABB(const Sphere& s, const Eigen::Vector3f& d, const AABB& b, float &t)
{
    // Compute the AABB resulting from expanding b by sphere radius r
    AABB e = b;
    e.m_min.array() -= s.m_radius;
    e.m_max.array() += s.m_radius;

    // Intersect ray against expanded AABB e. Exit with no intersection if ray
    // misses e, else get intersection point p and time t as result
    Eigen::Vector3f p;
    if (!IntersectRayAABB(s.m_pos, d, e, t, p) || t > 1.0f)
        return 0;

    // Compute which min and max faces of b the intersection point p lies
    // outside of. Note, u and v cannot have the same bits set and
    // they must have at least one bit set amongst them
    int u = 0, v = 0;
    if (p.x() < b.m_min.x()) u |= 1;
    if (p.x() > b.m_max.x()) v |= 1;
    if (p.y() < b.m_min.y()) u |= 2;
    if (p.y() > b.m_max.y()) v |= 2;
    if (p.z() < b.m_min.z()) u |= 4;
    if (p.z() > b.m_max.z()) v |= 4;

    // ‘Or’ all set bits together into a bit mask (note: here u + v == u | v)
    int m = u + v;

    // Define line segment [c, c+d] specified by the sphere movement
    const auto segA = s.m_pos;
    const auto segB = s.m_pos + d;

    // If all 3 bits set (m == 7) then p is in a vertex region
    if (m == 7) {
        // Must now intersect segment [c, c+d] against the capsules of the three
        // edges meeting at the vertex and return the best time, if one or more hit
        float tmin = FLT_MAX;
        if (IntersectSegmentCapsule(segA, segB, Corner(b, v), Corner(b, v ^ 1), s.m_radius, t))
            tmin = std::min(t, tmin);
        if (IntersectSegmentCapsule(segA, segB, Corner(b, v), Corner(b, v ^ 2), s.m_radius, t))
            tmin = std::min(t, tmin);
        if (IntersectSegmentCapsule(segA, segB, Corner(b, v), Corner(b, v ^ 4), s.m_radius, t))
            tmin = std::min(t, tmin);
        if (tmin == FLT_MAX) return 0; // No intersection
        t = tmin;
        return 1; // Intersection at time t == tmin
    }
    // If only one bit set in m, then p is in a face region
    if ((m & (m - 1)) == 0) {
        // Do nothing. Time t from intersection with
        // expanded box is correct intersection time
        return 1;
    }
    // p is in an edge region. Intersect against the capsule at the edge
    return IntersectSegmentCapsule(segA, segB, Corner(b, u ^ 7), Corner(b, v), s.m_radius, t);
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
    const auto vec = util::direction(sphere.m_pos, pA);
    pS = sphere.m_pos + vec * sphere.m_radius;
    return util::isNegative(dist);
}

inline bool intersect(const Capsule& capsule, const AABB& aabb, Eigen::Vector3f& pC, Eigen::Vector3f& pA, float& dist)
{
    float t;
    const Sphere s(capsule.m_min, capsule.m_radius);
    const auto sv = capsule.m_max - capsule.m_min;
    const auto intersects = IntersectMovingSphereAABB(s, sv, aabb, t);
    pC = s.m_pos + sv * t;
    ClosestPtPointAABB(pC, aabb.m_min, aabb.m_max, dist, pA);
    return intersects == 0 ? false : true;
}

inline void sortInterval(float& a1, float& a2)
{
    if (a1 > a2)
    {
        std::swap(a1, a2);
    }
}

inline bool intersectIntervals(float a1, float a2, float b1, float b2, float& ca, float& cb, float& dist)
{
    sortInterval(a1, a2);
    sortInterval(b1, b2);
    
    // first interval to the left of second
    if (a2 <= b1)
    {
        ca = a2;
        cb = b1;
        dist = b1 - a2;
        return false;
    }

    // first interval to the right of second
    if (a1 >= b2)
    {
        ca = a1;
        cb = b2;
        dist = a1 - b2;
        return false;
    }

    // from here on, we know the intervals intersect so the distances will be negative

    // first interval entirely inside of second or vice versa
    if ((a1 > b1 && a2 < b2) || (b1 > a1 && b2 < a2))
    {
        // distance between starting points of intervals
        float d1 = a1 - b1;
        // distance between ending points of intervals
        float d2 = b2 - a2;
        // the smaller distance is what we care for
        if (d1 < d2)
        {
            ca = a1;
            cb = b1;
            dist = -d1;
            return true;
        }
        else
        {
            ca = a2;
            cb = b2;
            dist = -d2;
            return true;
        }        
    }

    // first interval overlaps from the left the second one
    if (a2 > b1 && a2 < b2)
    {
        // distance between ending point of first interval and starting point of second interval
        dist = -(a2 - b1);
        // first interval penetrated into second up to its ending point
        ca = a2;
        // second interval collision point was the starting point
        cb = b1;
        return true;
    }

    // one last case remains, when the first interval overlaps on the right the second one
    // it is the opposite of the previous case
    dist = -(b2 - a1);
    ca = a1;
    cb = b2;
    return true;
}

inline bool intersect(const AABB& aabb1, const AABB& aabb2, Eigen::Vector3f& pA1, Eigen::Vector3f& pA2, float& dist)
{
#if 1
    bool intersects = true;    
    dist = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i)
    {
        float distAxis;
        bool intersectsAxis = intersectIntervals(aabb1.m_min[i], aabb1.m_max[i], aabb2.m_min[i], aabb2.m_max[i], pA1[i], pA2[i], distAxis);
        intersects = intersects && intersectsAxis;
        if (distAxis < dist)
        {
            dist = distAxis;
        }
    }
    return intersects;
#else

    float ca, cb;
    float a1 = 2;
    float a2 = 10;
    float b1 = -3;
    float b2 = 3;

    bool ii = intersectIntervals(a1, a2, b1, b2, ca, cb, dist);
    b1 -= dist;
    b2 -= dist;
    ii = intersectIntervals(a1, a2, b1, b2, ca, cb, dist);
    assert(ii == false);

    return false;
#endif
}
