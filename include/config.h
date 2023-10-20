#include <string>
#include <rrt_planner/utils.h>

typedef struct {
    std::string map_file;
    double goal_tolerance;
    double step;
    unsigned int min_num_nodes;
    unsigned int max_num_nodes;
    double goal_bias;

    double initial_x, initial_y;
    double goal_x, goal_y;
} config_t;

config_t get_config(const std::string filename);

rrt_planner::rrt_params config_to_params(config_t);
