#include <SDL2/SDL.h>

class Display
{
	public:
		Display(int width, int height);
		void clearWindow();
		void draw();
		void drawParticle(int x, int y, float radius, SDL_Color color);
		void drawCircle(int32_t centreX, int32_t centreY, int32_t radius, SDL_Color color);
		void getWindowSize(int *w, int *h);
		SDL_Color lerpColor(SDL_Color a, SDL_Color b, float t);
	private:
		SDL_Window* win;
		SDL_Renderer* renderer;
};