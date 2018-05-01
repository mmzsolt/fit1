#include <iostream>
#include "SDL.h"
#include "SDL_mouse.h"
#include "ray/camera.hpp"

SDL_Window *mainWindow;
SDL_Surface* screenSurface = NULL;

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
		512, 512, SDL_WINDOW_SHOWN);

	if (!mainWindow)
	{
		std::cout << "Unable to create window\n";
		CheckSDLError(__LINE__);
		return false;
	}


	SDL_SetRelativeMouseMode(SDL_TRUE);

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
		SDL_FillRect( screenSurface, NULL, SDL_MapRGB( screenSurface->format, 0x0, 0xFF, 0xFF ) );
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
