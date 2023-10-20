
#include <rrt_planner/collision_detector.h>


namespace rrt_planner {

    CollisionDetector::CollisionDetector(const pgm_t *map) {
        this->map = map;
        this->resolution_ = 0.05;
    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {
        unsigned int mx, my;

        // Convert world coordinates to map coordinates
        if (!map->worldToMap(world_pos[0], world_pos[1], mx, my)) {
            // Coordinates outside the map, consider it as obstacle
            return false;
        }

        // Get the cost at the specified map coordinates
        unsigned char cost = map->getCost(mx, my);

        return cost < 10;
    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {

        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return ( !inFreeSpace(point_b) ) ? true : false;

        } else {
            
            int num_steps = static_cast<int>(floor(dist/resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {

                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }

    }

};
