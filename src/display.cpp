#include <SDL2/SDL.h>
#include "display.h"
#include <iostream>

SDL_Window* win;
SDL_Renderer* renderer;

Display::Display(int width, int height) 
{
	SDL_Init(SDL_INIT_EVERYTHING);
	win = SDL_CreateWindow("Test", 100, 100, width, height, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
	if (!win)
		printf("Failed to create a window! Error: ", SDL_GetError());
	
	renderer = SDL_CreateRenderer(win, -1, 0);
	if (!renderer)
		printf("Failed to create a renderer! Error: ", SDL_GetError());
}

void Display::clearWindow() 
{
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
	SDL_RenderClear(renderer);
}

void Display::drawParticle(int x, int y, float radius, SDL_Color color)
{
	SDL_SetRenderDrawColor(renderer, color.r, color.b, color.g, color.a);
	for (int w = -radius; w <= radius; w++) {
		for (int h = -radius; h <= radius; h++) {
			if (w*w + h*h <= radius*radius)
				SDL_RenderDrawPoint(renderer, x + w, y + h);
		}
	}
}

void Display::draw() 
{
	SDL_RenderPresent(renderer);
}

void Display::getWindowSize(int* w, int* h) 
{
	SDL_GetWindowSize(win, w, h);
}

SDL_Color Display::lerpColor(SDL_Color a, SDL_Color b, float t)
{
	if (t < 0.0f) t = 0.0f;
	if (t > 1.0f) t = 1.0f;

	SDL_Color result;

	result.r = (Uint8)(a.r + (b.r - a.r) * t);
	result.g = (Uint8)(a.g + (b.g - a.g) * t);
	result.b = (Uint8)(a.b + (b.b - a.b) * t);
	result.a = (Uint8)(a.a + (b.a - a.a) * t);

	return result;
}

void Display::drawCircle(int32_t centreX, int32_t centreY, int32_t radius, SDL_Color color)
{
	SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

	const int32_t diameter = (radius * 2);

	int32_t x = (radius - 1);
	int32_t y = 0;
	int32_t tx = 1;
	int32_t ty = 1;
	int32_t error = (tx - diameter);

	while (x >= y)
	{
		//  Each of the following renders an octant of the circle
		SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
		SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
		SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
		SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
		SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
		SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
		SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
		SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

		if (error <= 0)
		{
			++y;
			error += ty;
			ty += 2;
		}

		if (error > 0)
		{
			--x;
			tx += 2;
			error += (tx - diameter);
		}
	}
}