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

void Display::drawParticle(int x, int y, float radius) 
{
	SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
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