#include <observer.h>
#include <error.h>
#include <rrt_planner/rrt_planner.h>
#include <draw.h>

RRTObserver::RRTObserver(SDL_Renderer *r, rrt_planner::RRTPlanner *planner, double zoom) {
    this->r = r;
    this->planner = planner;
    this->zoom = zoom;
}

void RRTObserver::draw(const pgm_t *pgm) {
    const std::vector<rrt_planner::Node> tree = planner->getTree();

    SDL_SetRenderDrawColor(this->r, 80, 80, 80, 200);

    for (size_t i = 0; i < tree.size(); i++) {
        auto node = tree[i];
        unsigned mx, my;

        pgm->worldToMap(node.pos[0], node.pos[1], mx, my);

        sdl_circle(this->r, mx*this->zoom, my*this->zoom, 2); 
    }
    
    if (!planner->path_found) return;

    SDL_SetRenderDrawColor(this->r, 0, 200, 20, 255);
    rrt_planner::Node current = tree[tree.size() - 1];
    while (current.parent_id != -1) {
        unsigned mx, my;
        unsigned pmx, pmy;

        rrt_planner::Node parent = tree[current.parent_id];

        pgm->worldToMap(current.pos[0], current.pos[1], mx, my);
        pgm->worldToMap(parent.pos[0], parent.pos[1], pmx, pmy);


        // THIIICK
        SDL_RenderDrawLine(this->r, mx*this->zoom, my*this->zoom, pmx*this->zoom, pmy*this->zoom);
        SDL_RenderDrawLine(this->r, 1 + mx*this->zoom, my*this->zoom, 1+ pmx *this->zoom, pmy*this->zoom);
        SDL_RenderDrawLine(this->r, mx*this->zoom, 1 + my*this->zoom, pmx *this->zoom, 1 + pmy*this->zoom);
        SDL_RenderDrawLine(this->r, 1 + mx*this->zoom, 1 + my*this->zoom, 1+ pmx *this->zoom, 1 + pmy*this->zoom);

        sdl_circle(this->r, mx*this->zoom, my*this->zoom, 2); 
        current = parent;
    }
}
