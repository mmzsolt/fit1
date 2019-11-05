#pragma once

#include "basic.hpp"

template <typename PRIMITIVE, typename LIGHT>
class Scene
{
public:
    Scene() {}
    ~Scene() {}
    using PrimPtr = std::shared_ptr<PRIMITIVE>;
    using PrimVec = std::vector<PrimPtr>;
    using LightPtr = std::shared_ptr<LIGHT>;
    using LightVec = std::vector<LightPtr>;
    PrimVec m_primitives;
    LightVec m_lights;

    Intersection intersectPrimitives(Ray ray, const Primitive* primitive, ShadowCastingType castingFilter)
    {
        Intersection bestIntersection;
        for (const auto& p : m_primitives)
        {
            if (p.get() == primitive)
            {
                continue;
            }

            if (castingFilter != ShadowCastingType::All && p->m_shadowCasting != castingFilter)
            {
                continue;
            }

            auto inters = p->intersect(ray);
            inters.m_primitive = p.get();
            if (inters.isValid() && inters.m_depth < bestIntersection.m_depth)
            {
                bestIntersection = inters;
            }
        }
        return bestIntersection;
    }

    template <typename CAM>
    void render(const CAM& camera, int* pixels, float* depths)
    {
    	if (m_primitives.empty())
	    {
		    return;
	    }
	
        for (int y = 0; y < camera.getHeight(); ++y)
        {
            for (int x = 0; x < camera.getWidth(); ++x)
            {
                auto cameraRay = camera.getRay(x, y);
                Intersection cameraIntersection = intersectPrimitives(cameraRay, nullptr, ShadowCastingType::All);
                if (cameraIntersection.isValid())
                {
                    Eigen::Vector3f finalColor(0.0f, 0.0f, 0.0f);
                    if (cameraIntersection.m_primitive->m_lighting == LightingType::Phong)
                    {
                        for(const auto& light : m_lights)
                        {
                            auto [lightDist, lightDir] = light->directionFrom(cameraIntersection.m_pos);
                            Ray lightRay(cameraIntersection.m_pos, lightDir);
                            Intersection lightIntersection = intersectPrimitives(lightRay, cameraIntersection.m_primitive, ShadowCastingType::Yes);
                            if (!lightIntersection.isValid() || lightIntersection.m_depth > lightDist)
                            {
                                auto color = light->color(cameraIntersection.m_normal, lightDir, -cameraRay.m_dir);
                                finalColor += color;
                            }
                        }
                    }
                    else if (cameraIntersection.m_primitive->m_lighting == LightingType::LightSource)
                    {
                        finalColor = {1.0f, 1.0f, 1.0f};
                    }
                    finalColor = finalColor.array() * cameraIntersection.m_primitive->m_color.array();
                    *depths = cameraIntersection.m_depth;
                    *pixels |= util::colorToRGB(finalColor);
                }
                ++depths;
                ++pixels;
            }
        }
    }

    void generateLightObjects()
    {
        for(const auto& light : m_lights)
        {            
            if (light->m_lightType != LightType::Point )
            {
                continue;
            }
            auto pointLight = static_cast<PointLight*>(light.get());
            auto sphere = std::make_shared<Sphere>(pointLight->m_pos, 0.1f);
            sphere->m_shadowCasting = ShadowCastingType::No;
            sphere->m_color = light->m_diffuse;
            sphere->m_lighting = LightingType::LightSource;
            m_primitives.push_back(sphere);
        }
    }

    void addIntersectionPoint(const Eigen::Vector3f& pos)
    {
        auto sphere = std::make_shared<Sphere>(pos, 0.1f);
        sphere->m_shadowCasting = ShadowCastingType::No;
        sphere->m_color = Eigen::Vector3f(0.0f, 1.0f, 1.0f);
        sphere->m_lighting = LightingType::LightSource;
        m_primitives.push_back(sphere);
    }
};
