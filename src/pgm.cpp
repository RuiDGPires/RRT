#include <pgm.h>
#include <iostream>
#include <limits>
#include <fstream>
#include <error.h>

int make_pixel(char c) {
    return c << 8 * 3 | c << 8 * 2 | c << 8  | 0xFF;
}

bool pgm_t::worldToMap(double wx, double wy, unsigned int &mx, unsigned int &my) const {
    
    int tmp_mx = wx / this->resolution;
    int tmp_my = wy / this->resolution;

    if (tmp_mx >= this->width || tmp_my >= this->height || tmp_mx < 0 || tmp_my < 0) {
        //ERROR("Out of bounds: (%u, %u)", tmp_mx, tmp_my);
        return false;
    }

    mx = (unsigned) tmp_mx;
    my = (unsigned) tmp_my;
        
    return true;
}

bool pgm_t::MapToWorld(unsigned mx, unsigned my, double &wx, double &wy) const {
    
    wx = mx * this->resolution;
    wy = my * this->resolution;

    return true;
}

unsigned char pgm_t::getCost(unsigned int x, unsigned int y) const {
    return 0xFF - ((this->pixels[y*this->width + x] >> 8) & 0xFF);
}

// Function to parse a PGM file and store pixel values in a vector
pgm_t parsePGM(const config_t& config) {
    std::ifstream file(config.map_file, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << config.map_file << std::endl;
        exit(1);
    }

    std::string magic;
    int width, height, maxVal;

    file >> magic ;
    file.ignore(); // Ignore newline character
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    file >> width >> height;
    file.ignore(); // Ignore newline character
    file >> maxVal;
    file.ignore(); // Ignore newline character

    if (magic != "P5" || maxVal > 255) {
        std::cerr << "Invalid P5 PGM file format." << std::endl;
        exit(1);
    }

    std::vector<unsigned int> pixels(width*height, 0);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            pixels[i*width + j] = make_pixel(file.get());
        }
    }

    return (pgm_t) {.resolution = config.resolution, .width = width, .height = height, .pixels = pixels};
}
