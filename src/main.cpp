#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include "display.h"
#include "vec2.h"

#define PI 3.14159265

using stclock = std::chrono::steady_clock;

struct Particle 
{
	vec2<float> pos;
	vec2<float> vel;
	vec2<float> accel;
	float density = 0.0;
};

float gravity = 25.0f;
float p_radius = 4.0f;
float mass = 100.0f;
float bounceStiffness = -0.5f;
float s_radius = 20.0f;
float viscosityCoeff = 250.0f;

int window_width = 800;
int window_height = 600;

float pressureConst = 10000.0f;
float targetDensity = 0.5f;

const int spacing = 10;
const int gridWidth = 30;
const int gridHeight = 30;

Particle particles[gridWidth * gridHeight];

Display display(window_width, window_height);

void boundsCheck(Particle& particle)
{
	if (particle.pos.y > window_height - p_radius) 
	{
		particle.pos.y = window_height - p_radius;
		particle.vel.y = particle.vel.y * bounceStiffness;
	}
	if (particle.pos.y < p_radius) 
	{
		particle.pos.y = p_radius;
		particle.vel.y = particle.vel.y * bounceStiffness;
	}
	if (particle.pos.x > window_width - p_radius) 
	{
		particle.pos.x = window_width - p_radius;
		particle.vel.x = particle.vel.x * bounceStiffness;
	}
	if (particle.pos.x < p_radius) 
	{
		particle.pos.x = p_radius;
		particle.vel.x = particle.vel.x * bounceStiffness;
	}
}

float pressureKernel(float r, float h) 
{
	if (r >= h) return 0.0f;
	
	float a = (h*h - r*r);
	
	return (4.0f / (PI * pow(h, 8))) * a*a*a;
}

vec2<float> pressureGradient(vec2<float> rij, float h) 
{
	float r = rij.length();
	
	if (r <= 0.0001f || r > h) return vec2<float>(0.0f, 0.0f);
	
	vec2<float> dir = rij / r;
	float derivative = -(30.0f / (PI * pow(h, 5))) * pow(h - r, 2);
	
	return dir * derivative;
}

float viscosityKernel(float r, float h) 
{
	if (r >= h) return 0.0f;
	return (40.0f / (PI * pow(h, 5))) * (h - r);
}

float getDensity(float x, float y, float smoothRadius)
{
	float density = 0.0f;
	
	for (auto& p : particles) 
	{
		float dx = p.pos.x - x;
		float dy = p.pos.y - y;
		float r = std::sqrt(dx * dx + dy * dy);
		if (r > smoothRadius) continue;
		
		density += mass * pressureKernel(r, smoothRadius);
	}
	
	return density;
}

float getPressure(float density) 
{
	return pressureConst * std::max(0.0f, density - targetDensity);
}

vec2<float> getPressureForce(Particle& particle)
{
	vec2<float> pressureForce;
	
	float pressure1 = getPressure(particle.density);
	float dens1 = std::max(particle.density, 0.0001f);
	
	for (auto& p : particles)
	{
		if (&p == &particle) continue;

		vec2<float> rij = 
		{
			particle.pos.x - p.pos.x,
			particle.pos.y - p.pos.y
		};
		
		float r = rij.length();
		
		if (r > s_radius || r <= 0.0f) continue;
		
		float pressure2 = getPressure(p.density);
		float dens2 = std::max(p.density, 0.0001f);
		
		float scalar = -mass * (pressure1/(dens1*dens1) + pressure2/(dens2*dens2));
		vec2<float> gradW = pressureGradient(rij, s_radius);
		
		pressureForce += gradW * scalar;
	}
	return pressureForce;
}

vec2<float> getViscosityForce(Particle& particle) 
{
	vec2<float> viscForce;
	
	for (auto& p : particles) 
	{
		if (&p == &particle) continue;
		
		vec2<float> rij = p.pos - particle.pos;
		float r = rij.length();
		if (r > s_radius || r <= 0.0f) continue;
		
		vec2<float> velDiff = p.vel - particle.vel;
		float laplacian = viscosityKernel(r, s_radius);
		
		float scalar = (mass / std::max(p.density, 0.0001f)) * laplacian * viscosityCoeff;
		
		viscForce += velDiff * scalar;
	}
	return viscForce;
}

void update(double delta) 
{
	display.clearWindow();
	
	for (auto& p : particles) 
	{
		p.density = getDensity(p.pos.x, p.pos.y, s_radius);

	}
	
	for (auto& p : particles) 
	{	
		p.accel = vec2<float>(0.0f, 0.0f);
		p.accel += getPressureForce(p);
		p.accel += getViscosityForce(p);
		p.accel.y += gravity;
	}
	
	for (auto& p : particles) 
	{	
		p.vel += p.accel * delta;
		p.pos += p.vel * delta;
		
		boundsCheck(p);
		display.drawParticle(p.pos.x, p.pos.y, p_radius);
	}

	display.draw();
}

int main() 
{
	auto previous = stclock::now();
	
	int quit = 0;
	SDL_Event event;
	
	for (int i = 0; i < gridWidth; i++) 
	{
		for (int j = 0; j < gridHeight; j++) 
		{
			int idx = j * gridWidth + i;
			particles[idx].pos.x = (window_width  - 9 * spacing) / 2.0f + i * spacing;
			particles[idx].pos.y = (window_height  - 9 * spacing) / 2.0f + j * spacing;
		}
	}
	
	while(!quit) 
	{
		display.getWindowSize(&window_width, &window_height);
		
		auto current = stclock::now();
		double delta = std::chrono::duration<double>(current - previous).count();
		previous = current;

		delta = std::min(delta, 0.05);
		
		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_QUIT)
				quit = 1;
		}
		
		update(delta);
		printf("%.2f\n", (float)(1.0f / delta));
		//printf("%f\n", particles[0].density);
	}
	SDL_Quit();
	return 0;
}