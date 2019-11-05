#include <iostream>
#include "SDL.h"
#include "SDL_mouse.h"
#include "ray/camera.hpp"
#include "ray/sphere.hpp"
#include "ray/triangle.hpp"
#include "ray/aabb.hpp"
#include "ray/capsule.hpp"
#include <vector>
#include <memory>
#include "ray/intersections.hpp"
#include "ray/light.hpp"
#include "ray/scene.hpp"

using SpherePtr = std::shared_ptr<Sphere>;
using CapsulePtr = std::shared_ptr<Capsule>;

SDL_Window *mainWindow;
SDL_Surface* screenSurface = NULL;
SDL_Surface* image = NULL;
std::vector<float> imageDepth;
Camera cam;
Camera_fi cam_fi;
std::vector<Sphere> spheres;
std::vector<Triangle> triangles;
std::vector<AABB> aabbs;
std::vector<Sphere_fi> spheres_fi;
std::vector<Triangle_fi> triangles_fi;

Scene<Primitive, Light> scene;

int screenWidth = 512;
int screenHeight = 512;

template <typename CAM, typename VEC>
void render(const CAM& camera, const VEC& prim)
{
	if (prim.empty())
	{
		return;
	}

	int* pixels = static_cast<int*>(image->pixels);
	auto depths = imageDepth.begin();
	
	for (int y = 0; y < image->h; ++y)
	{
		for (int x = 0; x < image->w; ++x)
		{
			auto r = camera.getRay(x, y);
			for (const auto& p : prim)
			{
				auto inters = p.intersect(r);
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

template<class T>
struct is_vector : std::false_type {};

template<class T>
struct is_vector<std::vector<T>> : std::true_type {};

template <typename VEC>
std::enable_if_t<is_vector<VEC>::value>
print(const VEC& prim)
{
	std::string toPrint;
	for (const auto& p : prim)
	{
		toPrint = p.toString();
		printf("%s\n", toPrint.c_str());
	}
}

template <typename PRIM>
std::enable_if_t<std::is_same<PRIM, Sphere>::value || std::is_same<PRIM, Triangle>::value>
print(const PRIM& prim)
{
	auto toPrint = prim.toString();
	printf("%s\n", toPrint.c_str());
}

void createScene()
{
	/*
	{
		Eigen::Vector3f pos(0.0f, 0.0f, 10.0f);
		Sphere sph{ pos, 3.0f };
		spheres.push_back(sph);
	}
	*/
	/*
	{
		Eigen::Vector3f pos1(0.0f, -1.0f, 10.0f);
		Eigen::Vector3f pos2(-1.0f, 1.0f, 10.0f);
		Eigen::Vector3f pos3(1.0f, 1.0f, 10.0f);
		Triangle tri{ pos1, pos2, pos3 };
		triangles.push_back(tri);
		auto quad = makeQuad({ 0.0f, 3.0f, 10.0f }, 1.0f, 4.0f);		
		triangles.push_back(quad.first);
		triangles.push_back(quad.second);
	}
	*/
	{
		Eigen::Vector3f pos1(-3.0f, -3.0f, 10.0f);
		Eigen::Vector3f pos2(-1.0f, -1.0f, 20.0f);
		AABB aabb{ pos1, pos2 };
		aabbs.push_back(aabb);
	}
}

void createScene_fi()
{
	{
		vec3fi pos(0.0f, 0.0f, 10.0f);
		Sphere_fi sph{ pos, 3.0f };
		spheres_fi.push_back(sph);
	}
	/*
	{
		vec3fi pos1(0.0f, -1.0f, 10.0f);
		vec3fi pos2(-1.0f, 1.0f, 10.0f);
		vec3fi pos3(1.0f, 1.0f, 10.0f);
		Triangle_fi tri{ pos1, pos2, pos3 };
		triangles_fi.push_back(tri);
		auto quad = makeQuad_fi({ 0.0f, 3.0f, 10.0f }, 1.0f, 4.0f);		
		triangles_fi.push_back(quad.first);
		triangles_fi.push_back(quad.second);
	}
	*/
}

void createScene2()
{
	SpherePtr sphere;
	SpherePtr sphere2;
	CapsulePtr capsule;
	{
		Eigen::Vector3f pos(-3.0f, -2.0f, 10.0f);
		sphere = std::make_shared<Sphere>(pos, 1.0f);
		scene.m_primitives.push_back(sphere);		
	}
	{
		Eigen::Vector3f pos(-2.0f, 3.0f, 10.0f);
		sphere2 = std::make_shared<Sphere>(pos, 0.5f);
		scene.m_primitives.push_back(sphere2);
	}
	/*
	{
		Eigen::Vector3f pos1(-5.0f, -3.0f, 10.0f);
		Eigen::Vector3f pos2(-3.0f, -1.0f, 20.0f);
		auto aabb = std::make_shared<AABB>(pos1, pos2);
		scene.m_primitives.push_back(aabb);
	}
	*/
	{
		Eigen::Vector3f pos1(0.0f, -1.0f, 10.0f);
		Eigen::Vector3f pos2(0.0f, 1.0f, 10.0f);
		capsule = std::make_shared<Capsule>(pos1, pos2, 1.0f);
		scene.m_primitives.push_back(capsule);
	}
	{
		Eigen::Vector3f pos(2.0f, 0.0f, 5.0f);
		auto light = std::make_shared<PointLight>(pos, 100.0f);
		light->m_diffuse = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
		light->m_shininess = 8.0f;		
		scene.m_lights.push_back(light);
		pos.x() = -2.0f;
		light = std::make_shared<PointLight>(pos, 100.f);
		light->m_diffuse = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
		light->m_shininess = 12.0f;
		scene.m_lights.push_back(light);
	}
	{
		auto quad = makeQuad({ 0.0f, 0.0f, 15.0f }, 10.0f, 10.0f);		
		quad.first.m_color = Eigen::Vector3f(0.2f, 1.0f, 0.3f);
		quad.second.m_color = Eigen::Vector3f(0.2f, 1.0f, 0.3f);
		//scene.m_primitives.push_back(std::make_shared<Triangle>(quad.first));
		//scene.m_primitives.push_back(std::make_shared<Triangle>(quad.second));
	}

	Eigen::Vector3f p1, p2;
	float dist;
	intersect(*capsule, *sphere, p1, p2, dist);
	scene.addIntersectionPoint(p1);
	scene.addIntersectionPoint(p2);

	intersect(*sphere2, *sphere, p1, p2, dist);
	scene.addIntersectionPoint(p1);
	scene.addIntersectionPoint(p2);

	scene.generateLightObjects();
}

// taken from http://headerphile.com/sdl2/opengl-part-1-sdl-opengl-awesome/
void CheckSDLError(int line = -1)
{
	std::string error = SDL_GetError();

	if (error != "")
	{
		std::cout << "SLD Error : " << error.c_str() << std::endl;

		if (line != -1)
			std::cout << "\nLine : " << line << std::endl;

		SDL_ClearError();
	}
}

bool Init()
{
	if (SDL_Init(SDL_INIT_VIDEO) < 0)
	{
		std::cout << "Failed to init SDL\n";
		return false;
	}

	mainWindow = SDL_CreateWindow("vox", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		screenWidth, screenHeight, SDL_WINDOW_SHOWN);

	if (!mainWindow)
	{
		std::cout << "Unable to create window\n";
		CheckSDLError(__LINE__);
		return false;
	}

	SDL_SetRelativeMouseMode(SDL_TRUE);

	Uint32 rmask, gmask, bmask, amask;

#if SDL_BYTEORDER == SDL_BIG_ENDIAN
	rmask = 0xff000000;
	gmask = 0x00ff0000;
	bmask = 0x0000ff00;
	amask = 0x000000ff;
#else
	rmask = 0x000000ff;
	gmask = 0x0000ff00;
	bmask = 0x00ff0000;
	amask = 0xff000000;
#endif

	image = SDL_CreateRGBSurface(0, screenWidth, screenHeight, 32, rmask, gmask, bmask, amask);

	if (!image)
	{
		std::cout << "Unable to create image\n";
		CheckSDLError(__LINE__);
		return false;
	}

	imageDepth.resize(screenWidth * screenHeight);

	cam.setWidth(static_cast<float>(screenWidth));
	cam.setHeight(static_cast<float>(screenHeight));
	cam_fi.setWidth(screenWidth);
	cam_fi.setHeight(screenHeight);

	createScene2();
	//createScene();
	//createScene_fi();

	return true;
}

void Run()
{
	bool loop = true;

	while (loop)
	{
		//Get window surface 
		screenSurface = SDL_GetWindowSurface( mainWindow );
		//Fill the surface white 
		//SDL_FillRect( screenSurface, NULL, SDL_MapRGB( screenSurface->format, 0x0, 0xFF, 0xFF ) );
		
		std::fill_n((int *)image->pixels, image->w * image->h, 0xff000000);
		std::fill_n(imageDepth.begin(), imageDepth.size(), FLT_MAX);

		/*
		render(cam, spheres);
		render(cam, triangles);
		render(cam, aabbs);
		render(cam_fi, spheres_fi);
		render(cam_fi, triangles_fi);
		*/
		scene.render(cam, (int *)image->pixels, &imageDepth[0]);

		SDL_BlitSurface(image, NULL, screenSurface, NULL);

		//Update the surface
		SDL_UpdateWindowSurface( mainWindow );

		SDL_Event event;

		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_QUIT)
				loop = false;

			if (event.type == SDL_KEYDOWN)
			{
				switch (event.key.keysym.sym)
				{
				case SDLK_ESCAPE:
					loop = false;
					break;
				}
			}
		
			if (event.type == SDL_MOUSEMOTION)
			{
			}
			
		}
	}
}

void Cleanup()
{
	SDL_DestroyWindow(mainWindow);

	SDL_FreeSurface(image);

	SDL_Quit();
}

#ifdef __linux__
int main(int argc, char *argv[])
#else
int SDL_main(int argc, char *argv[])
#endif
{
	if (!Init())
	{
		std::cout << "Could not init\n";
		return 1;
	}

	Run();

	Cleanup();

	return 0;
}
