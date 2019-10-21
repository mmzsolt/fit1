#include "basic_fixed.hpp"

vec3fi operator-(const vec3fi& a, const vec3fi& b)
{
    vec3fi ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

vec3fi operator+(const vec3fi& a, const vec3fi& b)
{
    vec3fi ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}