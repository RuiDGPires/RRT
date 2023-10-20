#pragma once
#include <SDL2/SDL.h>

//draw one quadrant arc, and mirror the other 4 quadrants
void sdl_ellipse(SDL_Renderer* r, int x0, int y0, int radiusX, int radiusY);
void sdl_circle(SDL_Renderer* r, int x0, int y0, int radiusX, bool filled = true);
