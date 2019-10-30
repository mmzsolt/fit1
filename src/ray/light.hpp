#pragma once

class Light
{
public:
    Light() {}
    virtual ~Light() {}
   	Eigen::Vector3f m_diffuse = {1.0f, 1.0f, 1.0f};
   	Eigen::Vector3f m_specular = {1.0f, 1.0f, 1.0f};
    float m_specularMultiplier = 5.0f;
    float m_diffuseMultiplier = 5.0f;
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
        float shiny = std::pow(cameraDir.dot(reflectDir), m_shininess);
        res += shiny * m_specularMultiplier * m_specular;
        return res;
    }
    virtual Eigen::Vector3f directionFrom(const Eigen::Vector3f& pos) const = 0;
};

class PointLight: public Light
{
public:
    PointLight() : Light() {}
    PointLight(Eigen::Vector3f _pos, float _radius) : Light(), m_pos(_pos), m_radius(_radius) {}
    ~PointLight() {}
    Eigen::Vector3f m_pos;
    float m_radius;
    virtual Eigen::Vector3f directionFrom(const Eigen::Vector3f& pos) const override
    {
        auto res = util::direction(pos, m_pos);
        return res;
    }
};