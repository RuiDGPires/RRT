#include <iostream>
#include <json.hpp>
#include <fstream>
#include <config.h>
#include <error.h>

using json = nlohmann::json;

#define OPT_PARAM(name, default_) { \
  if (json_config.contains(#name)) \
    json_config.at(#name).get_to(config.name); \
  else \
    config.name = default_; \
}


#define PARAM(name) { \
  if (json_config.contains(#name)) \
    json_config.at(#name).get_to(config.name); \
  else {\
    ERROR("Parameter missing: %s", #name); exit(0); \
  } \
}



config_t get_config(const std::string filename) {
  config_t config;
  json json_config = json::parse(std::ifstream(filename));

  PARAM(map_file);
  PARAM(goal_tolerance);
  PARAM(step);
  OPT_PARAM(min_num_nodes, 0);
  PARAM(max_num_nodes);
  PARAM(goal_bias);
  OPT_PARAM(goal_bias_adapt, false);
  OPT_PARAM(goal_bias_adapt_rate, 0.0);
  OPT_PARAM(goal_bias_adapt_reset, false);

  PARAM(initial_x);
  PARAM(initial_y);

  PARAM(goal_x);
  PARAM(goal_y);


  return config;
}

void print_config(config_t config) {
    std::cout << "Config:\n  " << config.map_file << "\n  " <<
    config.goal_tolerance << "\n  " <<
    config.step << "\n  " <<
    config.min_num_nodes << "\n  " <<
    config.max_num_nodes << "\n  " <<
    config.goal_bias << "\n  " <<
    config.goal_bias_adapt << "\n  " <<
    config.goal_bias_adapt_rate << "\n  " <<
    config.goal_bias_adapt_reset << "\n  " <<

    config.initial_x << "\n  " <<
    config.initial_y << "\n  " <<

    config.goal_x << "\n  " <<
    config.goal_y << "\n" << 
    std::endl;
}

rrt_planner::rrt_params config_to_params(config_t config) {
  return (rrt_planner::rrt_params) {
         .min_num_nodes = config.min_num_nodes,
         .max_num_nodes = config.max_num_nodes,
         .goal_tolerance = config.goal_tolerance,
         .step = config.step,
         .goal_bias = config.goal_bias,
  };
}
