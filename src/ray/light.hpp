#pragma once

enum class LightType { Point, Directional, Unset };

class Light
{
public:
    Light() {}
    virtual ~Light() {}
   	Eigen::Vector3f m_diffuse = {1.0f, 1.0f, 1.0f};
   	Eigen::Vector3f m_specular = {1.0f, 1.0f, 1.0f};
    float m_specularMultiplier = 0.5f;
    float m_diffuseMultiplier = 0.5f;
    float m_shininess = 3.0f;
    Eigen::Vector3f color(
        const Eigen::Vector3f& normal
        , const Eigen::Vector3f& lightDir
        , const Eigen::Vector3f& cameraDir) const
    {
        Eigen::Vector3f res(0.0f, 0.0f, 0.0f);
        float diffy = normal.dot(lightDir);
        if (diffy < 0.0f)
        {
            return res;
        }
        auto reflectDir = util::reflect(normal, lightDir);
        res = m_diffuseMultiplier * m_diffuse * diffy;
        float shiny = cameraDir.dot(reflectDir);
        if (shiny < 0.0f)
        {
            return res;
        }
        res += std::pow(shiny, m_shininess) * m_specularMultiplier * m_specular;
        return res;
    }
    virtual std::pair<float, Eigen::Vector3f> directionFrom(const Eigen::Vector3f& pos) const = 0;
    LightType m_lightType = LightType::Unset;
};

class PointLight: public Light
{
public:
    PointLight() : Light() { m_lightType = LightType::Point; }
    PointLight(Eigen::Vector3f _pos, float _radius) : Light(), m_pos(_pos), m_radius(_radius)    
    { m_lightType = LightType::Point; }
    ~PointLight() {}
    Eigen::Vector3f m_pos;
    float m_radius;
    virtual std::pair<float, Eigen::Vector3f> directionFrom(const Eigen::Vector3f& pos) const override
    {
        auto res = util::directionAndDistance(pos, m_pos);
        return res;
    }
};

class DirectionalLight: public Light
{
public:
    Eigen::Vector3f m_direction;
    DirectionalLight() : Light() { m_lightType = LightType::Directional; }
    DirectionalLight(decltype(m_direction) _direction) : m_direction(_direction)
    {
        m_lightType = LightType::Directional;
        normalizeDirection();
    }
    void normalizeDirection()
    {
        m_direction.normalize();
    }
    virtual std::pair<float, Eigen::Vector3f> directionFrom(const Eigen::Vector3f& pos) const override
    {        
        return std::make_pair(FLT_MAX, m_direction);
    }
};