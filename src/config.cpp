#include <json.hpp>
#include <fstream>
#include <config.h>

using json = nlohmann::json;

config_t get_config(const std::string filename) {
  config_t config;
  json json_config = json::parse(std::ifstream(filename));

  json_config.at("goal_tolerance").get_to(config.goal_tolerance);
  json_config.at("step").get_to(config.step);
  json_config.at("min_num_nodes").get_to(config.min_num_nodes);
  json_config.at("max_num_nodes").get_to(config.max_num_nodes);
  json_config.at("goal_bias").get_to(config.goal_bias);

  json_config.at("initial_x").get_to(config.initial_x);
  json_config.at("initial_y").get_to(config.initial_y);

  json_config.at("goal_x").get_to(config.goal_x);
  json_config.at("goal_y").get_to(config.goal_y);

  return config;
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
