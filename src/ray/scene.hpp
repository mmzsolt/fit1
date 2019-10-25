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
                auto r = camera.getRay(x, y);
                for (const auto& p : m_primitives)
                {
                    auto inters = p->intersect(r);
                    if (inters.isValid() && inters.m_depth < *depths)
                    {
                        *depths = static_cast<float>(inters.m_depth);
                        *pixels |= 0xffffff;
                    }
                }
                ++depths;
                ++pixels;
            }
        }
    }
};
