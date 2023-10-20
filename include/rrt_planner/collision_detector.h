
#ifndef _RTT_PLANNER_COLLISION_DETECTOR_H
#define _RTT_PLANNER_COLLISION_DETECTOR_H

#include "utils.h"
#include "point.h"
#include <vector>
#include <pgm.h>

namespace rrt_planner {

    class CollisionDetector {

        public:
            /**
             * @brief   Constructor for the CollisionDetector object
             * @param   costmap A pointer to the ROS wrapper of the costmap used for planning
             */ 
            explicit CollisionDetector(const pgm_t *map);

            /**
             * @brief   Examine whether world_pos is in free space 
             * @param   world_pos 2D array of 2D coordinates (x, y) expressed in the world frame
             * @return  True if yes, false otherwise
             */ 
            bool inFreeSpace(const double* world_pos);

            /**
             * @brief   Examine whether there is an obstacle between point_a and point_b
             * @param   point_a array of 2D coordinates (x, y) expressed in the world frame
             * @param   point_b array of 2D coordinates (x, y) expressed in the world frame
             */ 
            bool obstacleBetween(const double* point_a, const double* point_b);

        private:
            const pgm_t *map;
            double resolution_;
    };


};

#endif // _RTT_PLANNER_COLLISION_DETECTOR_H
