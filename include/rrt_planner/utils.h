
#ifndef _RTT_PLANNER_UTILS_H
#define _RTT_PLANNER_UTILS_H

#include <cmath>

namespace rrt_planner {

    struct rrt_params {
        unsigned int min_num_nodes;
        unsigned int max_num_nodes;
        double goal_tolerance;
        double step;
        double goal_bias;
        int goal_sample_thresh;
        bool   goal_bias_adapt;
        double goal_bias_adapt_rate;
        bool   goal_bias_adapt_reset;
        bool   settle_for_best;
        unsigned settle_for_best_tries;
        bool   settle_for_best_keep_best;
        bool   goal_check;
    };

    struct Node {
        double pos[2]; // 2D coordinates (x, y)
        int node_id;
        int parent_id;
        float cost_to_go{0.0};

        Node() {}

        Node(const double *pos_, int node_index, int parent_index) : 
                pos{pos_[0], pos_[1]}, node_id(node_index), parent_id(parent_index) {}

        bool operator ==(const Node& node) { return node_id == node.node_id; }

        bool operator !=(const Node& node) { return !(node_id == node.node_id); }
    };

    inline double computeDistance(const double *x, const double *y) {
        return std::hypot((y[1] - x[1]), (y[0] - x[0])); 
    }

    inline double computeDistanceSqrd(const double *x, const double *y) {
        double a = (y[1]-x[1]);
        double b = (y[0]-x[0]);
        return a*a + b*b;
    }
};

#endif // _RTT_PLANNER_UTILS_H
