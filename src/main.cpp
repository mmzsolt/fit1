#include <iostream>
#include "SDL.h"
#include "SDL_mouse.h"
#include "ray/camera.hpp"
#include "ray/sphere.hpp"
#include <vector>

SDL_Window *mainWindow;
SDL_Surface* screenSurface = NULL;
SDL_Surface* image = NULL;
std::vector<float> imageDepth;
Camera cam;
std::vector<Sphere> spheres;

int screenWidth = 512;
int screenHeight = 512;

template <typename VEC>
void render(const VEC& prim)
{
	int* pixels = static_cast<int*>(image->pixels);
	auto depths = imageDepth.begin();
	
	for (int y = 0; y < image->h; ++y)
	{
		for (int x = 0; x < image->w; ++x)
		{
			Ray r = cam.getRay(static_cast<float>(x), static_cast<float>(y));
			for (const auto& p : prim)
			{
				Intersection inters = p.intersect(r);
				if (inters.isValid() && inters.m_depth < *depths)
				{
					*depths = inters.m_depth;
					*pixels |= 0xffffff;
				}
			}
			++depths;
			++pixels;
		}
	}
}

void createScene()
{
	Eigen::Vector3f pos(0.0f, 0.0f, 10.0f);
	Sphere sph{ pos, 3.0f };
	spheres.push_back(sph);
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

	createScene();

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

		render(spheres);

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

int SDL_main(int argc, char *argv[])
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
