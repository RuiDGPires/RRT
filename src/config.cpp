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

  OPT_PARAM(description, "");
  PARAM(map_file);
  OPT_PARAM(resolution, 0.05);
  PARAM(goal_tolerance);
  PARAM(step);
  OPT_PARAM(min_num_nodes, 0);
  PARAM(max_num_nodes);
  PARAM(goal_bias);
  OPT_PARAM(goal_chance, 0.0);
  OPT_PARAM(goal_bias_adapt, false);
  OPT_PARAM(goal_bias_adapt_rate, 0.0);
  OPT_PARAM(goal_bias_adapt_reset, false);
  OPT_PARAM(settle_for_best, false);
  OPT_PARAM(settle_for_best_tries, 0);
  OPT_PARAM(settle_for_best_keep_best, false);

  PARAM(initial_x);
  PARAM(initial_y);

  PARAM(goal_x);
  PARAM(goal_y);

  return config;
}

#define STREAM_PARAM(param) " - " << #param << ": " << config.param << "\n" <<

void print_config(config_t config) {
    std::cout << 
    "\n [ " << config.description << " ]\n\n"
    STREAM_PARAM(map_file)
    STREAM_PARAM(resolution)
    STREAM_PARAM(goal_tolerance)
    STREAM_PARAM(step)
    STREAM_PARAM(min_num_nodes)
    STREAM_PARAM(max_num_nodes)
    STREAM_PARAM(goal_bias)
    STREAM_PARAM(goal_chance)
    STREAM_PARAM(goal_bias_adapt)
    STREAM_PARAM(goal_bias_adapt_rate)
    STREAM_PARAM(goal_bias_adapt_reset)
    STREAM_PARAM(settle_for_best)
    STREAM_PARAM(settle_for_best_tries)
    STREAM_PARAM(settle_for_best_keep_best)

    STREAM_PARAM(initial_x)
    STREAM_PARAM(initial_y)
    STREAM_PARAM(goal_x)
    STREAM_PARAM(goal_y)
    std::endl;
}

#define SET_PARAM(name) .name = config.name

rrt_planner::rrt_params config_to_params(config_t config) {
  return (rrt_planner::rrt_params) {
    SET_PARAM(min_num_nodes),
    SET_PARAM(max_num_nodes),
    SET_PARAM(goal_tolerance),
    SET_PARAM(step),
    SET_PARAM(goal_bias),
    .goal_sample_thresh = config.goal_chance == 0 ? -1: (int) (1.0 / config.goal_chance),
    SET_PARAM(goal_bias_adapt),
    SET_PARAM(goal_bias_adapt_rate),
    SET_PARAM(goal_bias_adapt_reset),
    SET_PARAM(settle_for_best),
    SET_PARAM(settle_for_best_tries),
    SET_PARAM(settle_for_best_keep_best),
  };
}
