
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
        tolerance_sqrd = params_.goal_tolerance * params_.goal_tolerance;
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();
        this->current_goal_bias = params_.goal_bias;

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;
        this->current_goal_bias = params_.goal_bias;
        this->best_node_dist_sqrd = std::numeric_limits<double>::max();
        this->best_node = -1;
        this->path_found = false;
        this->reached_goal = false;
        this->tries++;

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
                double dist_to_goal = computeDistanceSqrd(p_new, goal_);        

                if(dist_to_goal <= this->tolerance_sqrd){
                    this->path_found = true;
                    this->reached_goal = true;
                    this->tries = 0;
                    return true;
                } else if (sampleGoal(p_new)) {
                    this->path_found = true;
                    this->reached_goal = true;
                    this->tries = 0;
                    return true;
                } else if (dist_to_goal <= this->best_node_dist_sqrd && params_.settle_for_best) {
                    this->best_node_dist_sqrd = dist_to_goal;
                    this->best_node = nodes_.size() - 1;
                }
            }
        } 

        if (this->best_node != (unsigned) -1 && params_.settle_for_best && this->tries >= params_.settle_for_best_tries) {
            nodes_[nodes_.size() - 1] = nodes_[this->best_node];
            this->path_found = true;
            this->reached_goal = false;
            this->tries = 0;
            return true; 
        }

        return false;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {
        int nearest_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < nodes_.size(); i++) {
            double dist = computeDistanceSqrd(point, nodes_[i].pos);
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
        // goal_sample_thresh is used to take a sample of the goal point if n_samples surpasses it
        
        if ((int)++n_samples > params_.goal_sample_thresh && params_.goal_sample_thresh != -1) {
            n_samples = 0;
            rand_point_[0] = goal_[0];
            rand_point_[1] = goal_[1];
            return  rand_point_;
        }

        rand_point_[0] = random_double_x.generate() * (1 - this->current_goal_bias) + this->current_goal_bias * goal_[0];

        rand_point_[1] = random_double_y.generate() * (1 - this->current_goal_bias) + this->current_goal_bias * goal_[1];

        return rand_point_;
    }

    bool RRTPlanner::sampleGoal(double *pos) {
        if (!params_.goal_check) return false;

        if (!collision_dect_.obstacleBetween(pos, goal_)) {
            createNewNode(goal_, nodes_.size() - 1);
            return true;
        }

        return false;
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

    void RRTPlanner::move() {
        if (best_node == (unsigned) -1) return;

        this->setStart(nodes_[best_node].pos);
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];

    }

};
