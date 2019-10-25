#pragma once

class Light
{
public:
    Light() {}
    virtual ~Light() {}
   	Eigen::Vector3f m_color = {1.0f, 1.0f, 1.0f};
};

class PointLight: public Light
{
public:
    PointLight() : Light() {}
    ~PointLight() {}
};