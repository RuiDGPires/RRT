#pragma once
#include <SDL2/SDL.h>
#include <rrt_planner/rrt_planner.h>

class RRTObserver{
    private:
        SDL_Renderer *r;
        rrt_planner::RRTPlanner *planner;
        double zoom;

    public:
        RRTObserver(SDL_Renderer *r, rrt_planner::RRTPlanner *planner, double zoom);
        void draw(const pgm_t *);
};

