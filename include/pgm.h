#pragma once
#include <vector>
#include <string>

#define RESOLUTION 0.05

struct pgm_t {
    const int width, height;
    std::vector<unsigned int> pixels;

    unsigned char getCost(unsigned int x, unsigned int y) const ;
    bool worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const ;
    bool MapToWorld(unsigned mx, unsigned my, double &wx, double &wy) const ;
};

pgm_t parsePGM(const std::string& filename);
