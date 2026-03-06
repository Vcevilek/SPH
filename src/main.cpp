#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <unordered_map>
#include <thread>
#include "display.h"
#include "vec2.h"
#include <omp.h>

#define PI 3.14159265358979323846

using stclock = std::chrono::steady_clock;

struct Particle 
{
	vec2<float> pos;
	vec2<float> vel;
	vec2<float> accel;
	float density = 0.0f;
	float targetDensity = 0.0f;
	vec2<float> prevPos;
	SDL_Color color;
};

float gravity = 750.0f;
float p_radius = 2.0f;
float mass = 50.0f;
float bounceStiffness = -0.25f;
float s_radius = 25.0f;
float viscosityCoeff = 150.0f;

bool gravityEnabled = true;

int window_width = 800;
int window_height = 600;

float pressureConst = 51200.0f;
float particletargetDensity = 0.15f;

const int spacing = 10;
const int gridWidth = 50;
const int gridHeight = 50;

const int prime1 = 1014183103;
const int prime2 = 2007549391;

// Mouse variables
int mouseX = 0.0f;
int mouseY = 0.0f;
float m_radius = 50.0f;

bool showFPS = true;

Particle particles[gridWidth * gridHeight];
std::unordered_map<size_t, std::vector<int>> hashMap;

Display display(window_width, window_height);

void boundsCheck(Particle& particle)
{
	if (particle.pos.y > window_height - p_radius) 
	{
		particle.pos.y = window_height - p_radius;
		float vy = particle.pos.y - particle.prevPos.y;
		particle.prevPos.y = particle.pos.y + vy * bounceStiffness;
	}
	if (particle.pos.y < p_radius) 
	{
		particle.pos.y = p_radius;
		float vy = particle.pos.y - particle.prevPos.y;
		particle.prevPos.y = particle.pos.y + vy * bounceStiffness;
	}
	if (particle.pos.x > window_width - p_radius) 
	{
		particle.pos.x = window_width - p_radius;
		float vx = particle.pos.x - particle.prevPos.x;
		particle.prevPos.x = particle.pos.x + vx * bounceStiffness;
	}
	if (particle.pos.x < p_radius) 
	{
		particle.pos.x = p_radius;
		float vx = particle.pos.x - particle.prevPos.x;
		particle.prevPos.x = particle.pos.x + vx * bounceStiffness;
	}
}

float pressureKernel(float r, float h) 
{
	if (r >= h) return 0.0f;
	
	float a = (h*h - r*r);
	
	return (4.0f / (PI * pow(h, 8))) * a*a*a;
}

vec2<float> pressureGradient(vec2<float>& rij, float r2, float h) 
{
	if (r2 <= 0.0001f || r2 > h*h) return vec2<float>(0.0f, 0.0f);
	
	float r = std::sqrt(r2);
	vec2<float> dir = rij / r;
	float derivative = -(30.0f / (PI * pow(h, 5))) * pow(h - r, 2);
	
	return dir * derivative;
}

float viscosityKernel(float r, float h) 
{
	if (r >= h) return 0.0f;
	return (40.0f / (PI * pow(h, 5))) * (h - r);
}

float getDensity(Particle& particle, float smoothRadius)
{
	float density = 0.0f;
	float h2 = smoothRadius * smoothRadius;
	
	int cellX = floor(particle.pos.x / s_radius);
	int cellY = floor(particle.pos.y / s_radius);
	
	for (int dx = -1; dx <= 1; dx++) 
	{
		for (int dy = -1; dy <= 1; dy++) 
		{
			int neighborX = cellX + dx;
			int neighborY = cellY + dy;
			size_t hash = (neighborX * prime1) ^ (neighborY * prime2);
			
			auto it = hashMap.find(hash);
			if (it != hashMap.end()) 
			{
				for (int idx : it->second) 
				{
					auto& p = particles[idx];
					if (&p == &particle) continue;
					float diffX = p.pos.x - particle.pos.x;
					float diffY = p.pos.y - particle.pos.y;
					float r2 = diffX * diffX + diffY * diffY;
					if (r2 > h2) continue;
		
					density += mass * pressureKernel(std::sqrt(r2), smoothRadius);
				}
			}
		}
	}
	return density;
}

float getPressure(float density, float tDensity) 
{
	return pressureConst * (density - tDensity);
}

vec2<float> getPressureForce(Particle& particle)
{
	vec2<float> pressureForce;
	
	float pressure1 = getPressure(particle.density, particle.targetDensity);
	float dens1 = std::max(particle.density, 0.0001f);
	
	int cellX = floor(particle.pos.x / s_radius);
	int cellY = floor(particle.pos.y / s_radius);
	
	for (int dx = -1; dx <= 1; dx++) 
	{
		for (int dy = -1; dy <= 1; dy++) 
		{
			int neighborX = cellX + dx;
			int neighborY = cellY + dy;
			size_t hash = (neighborX * prime1) ^ (neighborY * prime2);
			
			auto it = hashMap.find(hash);
			if (it != hashMap.end()) 
			{
				for (int idx : it->second) 
				{
					auto& p = particles[idx];
					if (&p == &particle) continue;
					
					vec2<float> rij = particle.pos - p.pos;
		
					float r2 = rij.x*rij.x + rij.y*rij.y;
					float h2 = s_radius * s_radius;
					if (r2 > h2 || r2 <= 0.0001f) continue;
					
					float pressure2 = getPressure(p.density, p.targetDensity);
					float dens2 = std::max(p.density, 0.0001f);
		
					float scalar = -mass * (pressure1/(dens1*dens1) + pressure2/(dens2*dens2));
					vec2<float> gradW = pressureGradient(rij, r2, s_radius);
		
					pressureForce += gradW * scalar;
				}
			}
		}
	}
	return pressureForce;
}

vec2<float> getViscosityForce(Particle& particle) 
{
	vec2<float> viscForce;
	
	int cellX = floor(particle.pos.x / s_radius);
	int cellY = floor(particle.pos.y / s_radius);
	
	for (int dx = -1; dx <= 1; dx++) 
	{
		for (int dy = -1; dy <= 1; dy++) 
		{
			int neighborX = cellX + dx;
			int neighborY = cellY + dy;
			size_t hash = (neighborX * prime1) ^ (neighborY * prime2);
			
			auto it = hashMap.find(hash);
			if (it != hashMap.end()) 
			{
				for (int idx : it->second) 
				{
					auto& p = particles[idx];
					if (&p == &particle) continue;
		
					vec2<float> rij = p.pos - particle.pos;
					float r2 = rij.x*rij.x + rij.y*rij.y;
					float h2 = s_radius * s_radius;
					if (r2 > h2 || r2 <= 0.0f) continue;
		
					float r = std::sqrt(r2);
					
					vec2<float> velDiff = p.vel - particle.vel;
					float laplacian = viscosityKernel(r, s_radius);
		
					float scalar = (mass / std::max(p.density, 0.0001f)) * laplacian * viscosityCoeff;
		
					viscForce += velDiff * scalar;
				}
			}
		}
	}

	return viscForce;
}

vec2<float> getMouseForce(Particle& particle, Uint32 buttons) 
{
	if (!(buttons & (SDL_BUTTON(SDL_BUTTON_RIGHT) | SDL_BUTTON(SDL_BUTTON_LEFT)))) return vec2<float>(0.0f, 0.0f);
	
	vec2<float> mousePos(mouseX, mouseY);
	vec2<float> offset = mousePos - particle.pos;
	
	float dist2 = offset.x*offset.x + offset.y*offset.y;
	float radius2 = m_radius*m_radius;
	
	if(dist2 > radius2 || dist2 < 0.00001f) return vec2<float>(0.0f, 0.0f);
	
	float dist = std::sqrt(dist2);
	vec2<float> normalized = offset / dist;
	float centerT = pow(1.0f - dist / m_radius, 0.5f);
	
	int sign = 0;
	if (buttons & SDL_BUTTON(SDL_BUTTON_LEFT)) sign = 1;
	else if (buttons & SDL_BUTTON(SDL_BUTTON_RIGHT)) sign = -1;
	
	float strenght = 5000.0f;
	
	vec2<float> targetForce = normalized * (strenght * sign) - particle.vel;
	
	return targetForce * centerT;
}

void update(double delta) 
{
	display.clearWindow();
	
	Uint32 buttons = SDL_GetMouseState(&mouseX, &mouseY);
	
	hashMap.clear();
	
	for (int i = 0; i < gridHeight * gridWidth; i++) 
	{
		auto &p = particles[i];
		int cellX = floor(p.pos.x / s_radius);
		int cellY = floor(p.pos.y / s_radius);
		size_t hash = (cellX * prime1) ^ (cellY * prime2);
		hashMap[hash].push_back(i);
	}
	
	#pragma omp parallel
	{
		#pragma omp for schedule(static)
		for (int i = 0; i < gridHeight * gridWidth; i++) 
		{
			auto& p = particles[i];
			p.density = getDensity(p, s_radius);
		}
		
		#pragma omp for schedule(static)
		for (int i = 0; i < gridHeight * gridWidth; i++) 
		{
			auto& p = particles[i];
			p.accel = vec2<float>(0.0f, 0.0f);
			
			float maxForce = 10000.0f;
			vec2<float> pf = getPressureForce(p);
			if (pf.length() > maxForce) pf = pf.normalize() * maxForce;
			p.accel += pf;
			
			vec2<float> vf = getViscosityForce(p);
			if (vf.length() > maxForce) vf = vf.normalize() * maxForce;
			p.accel += vf;

			vec2<float> mf = getMouseForce(p, buttons);
			if (mf.length() > maxForce) mf = mf.normalize() * maxForce;
			p.accel += mf;
			
			if (gravityEnabled) p.accel.y += gravity;
		}
		
		#pragma omp for schedule(static)
		for (int i = 0; i < gridHeight * gridWidth; i++) 
		{	
			auto& p = particles[i];
			vec2<float> prev = p.pos;
			
			vec2<float> velocity = (p.pos - p.prevPos) * 0.99f;
			
			p.pos += velocity + p.accel * delta * delta;
			p.prevPos = prev;
			
			boundsCheck(p);
			
			p.vel = (p.pos - p.prevPos) / delta;
		}
	}
	
	for (auto& p : particles)
	{
		float factor = std::clamp(p.vel.length() / 200.0f, 0.0f, 1.0f);
		SDL_Color blue = {0, 255, 0, 255};
		SDL_Color red  = {255, 0, 0, 255};
		SDL_Color endColor = display.lerpColor(blue, red, factor);
		display.drawParticle(p.pos.x, p.pos.y, p_radius, endColor);
	}
	
	display.drawCircle(mouseX, mouseY, m_radius, {255, 255, 255, 255});
	display.draw();
}

int main() 
{
	auto previous = stclock::now();
	
	int quit = 0;
	SDL_Event event;
	
	hashMap.reserve(gridWidth * gridHeight);
	
	for (int i = 0; i < gridWidth; i++) 
	{
		for (int j = 0; j < gridHeight; j++) 
		{
			int idx = j * gridWidth + i;
			float x = window_width / 2.0f - (gridWidth / 2.0f * spacing) + spacing * i;
			float y = window_height / 2.0f - (gridHeight / 2.0f * spacing) + spacing * j;
			particles[idx].pos.x = x;
			particles[idx].pos.y = y;
			particles[idx].prevPos.x = x;
			particles[idx].prevPos.y = y;
			particles[idx].targetDensity = particletargetDensity;
			particles[idx].color = {255, 255, 255, 255};
		}
	}

	while(!quit) 
	{
		display.getWindowSize(&window_width, &window_height);
		
		auto current = stclock::now();
		double delta = std::chrono::duration<double>(current - previous).count();
		previous = current;

		delta = std::min(delta, 0.1);
		
		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_QUIT)
				quit = 1;
			if (event.type == SDL_MOUSEWHEEL) 
				m_radius += event.wheel.y * 2.0f;
		}
		
		update(delta);

		static double fpsAccum = 0.0;
		static int fpsCount = 0;
		fpsAccum += delta;
		fpsCount++;
		if (fpsAccum >= 0.5) 
		{
			if (showFPS) printf("FPS: %.1f\n", fpsCount / fpsAccum);
			fpsAccum = 0.0;
			fpsCount = 0;
		}
	}
	SDL_Quit();
	return 0;
}