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

    Intersection intersectPrimitives(Ray ray, const Primitive* primitive)
    {
        Intersection bestIntersection;
        for (const auto& p : m_primitives)
        {
            if (p.get() == primitive)
            {
                continue;
            }

            auto inters = p->intersect(ray);
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
                Intersection cameraIntersection = intersectPrimitives(cameraRay, nullptr);
                if (cameraIntersection.isValid())
                {
                    Eigen::Vector3f finalColor(0.0f, 0.0f, 0.0f);
                    for(const auto& light : m_lights)
                    {
                        auto lightDir = light->directionFrom(cameraIntersection.m_pos);
                        Ray lightRay(cameraIntersection.m_pos, lightDir);
                        Intersection lightIntersection = intersectPrimitives(lightRay, cameraIntersection.m_primitive);
                        if (!lightIntersection.isValid())
                        {
                            auto color = light->color(cameraIntersection.m_normal, lightDir, -cameraRay.m_dir);
                            finalColor += color;
                        }
                    }
                    *depths = cameraIntersection.m_depth;
                    *pixels |= util::colorToRGB(finalColor);
                }
                ++depths;
                ++pixels;
            }
        }
    }
};
