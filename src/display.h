#include <SDL2/SDL.h>

class Display
{
	public:
		Display(int width, int height);
		void clearWindow();
		void draw();
		void drawParticle(int x, int y, float radius);
		void getWindowSize(int *w, int *h);
	private:
		SDL_Window* win;
		SDL_Renderer* renderer;
};