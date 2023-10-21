
#include <observer.h>
#include <rrt_planner/rrt_planner.h>
#include <error.h>

double max(double a, double b) {
    if (a > b)
        return a;
    else
        return b;
}

double min(double a, double b) {
    if (a < b)
        return a;
    else
        return b;
}

namespace rrt_planner {

    RRTPlanner::RRTPlanner(const pgm_t *map, 
            const rrt_params& params) : map(map), params_(params), collision_dect_(map) {

        map->MapToWorld(map->width, map->height, map_width_, map_height_);

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;
        this->current_goal_bias = params_.goal_bias;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

                if  (params_.goal_bias_adapt) {
                    if (params_.goal_bias_adapt_reset) {
                        this->current_goal_bias = params_.goal_bias;
                    } else {
                        this->current_goal_bias = max(params_.goal_bias, this->current_goal_bias + params_.goal_bias_adapt_rate);
                    }
                }
            } else {
                if (params_.goal_bias_adapt)
                    this->current_goal_bias = min(0, this->current_goal_bias - params_.goal_bias_adapt_rate);
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){

                    this->path_found = true;
                    return true;
                }
            }
        } 

        return false;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {
        int nearest_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < nodes_.size(); i++) {
            double dist = computeDistance(point, nodes_[i].pos);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_id = i;
            }
        }
    
        if (nearest_id != -1) {
            return nearest_id;
        } else {
            ERROR("No nearest node");
            exit(0);
        }
    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) {

        Node new_node;

        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];

        new_node.node_id = nodes_.size();
        new_node.parent_id = parent_node_id;

        nodes_.emplace_back(new_node);

    }

    double* RRTPlanner::sampleRandomPoint() {

        // goal_bias is used to interpolate between the random generated point and the goal point by goal_bias

        rand_point_[0] = random_double_x.generate() * (1 - this->current_goal_bias) + this->current_goal_bias * goal_[0];

        rand_point_[1] = random_double_y.generate() * (1 - this->current_goal_bias) + this->current_goal_bias * goal_[1];

        return rand_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {

        double distance = computeDistance(point_nearest, point_rand);
        double t = params_.step / distance;

        candidate_point_[0] = point_nearest[0] + t * (point_rand[0] - point_nearest[0]);
        candidate_point_[1] = point_nearest[1] + t * (point_rand[1] - point_nearest[1]);

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];

    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];

    }

};
