
#include <rrt_planner/rrt_planner.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

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

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance){
                    //double path_length = computePathLength();
                    /*if(path_length < prev_path_length || prev_path_length == 0){
                        prev_path_length = path_length;
                        return true;
                    } */
                    return true;
                }
            }
        }

        return false;
    }

    double RRTPlanner::computePathLength(){
        double path_length = 0;
        int node_id = nodes_.size() - 1;
        while(node_id !=0){
            path_length += computeDistance(nodes_[node_id].pos, nodes_[nodes_[node_id].parent_id].pos);
            node_id = nodes_[node_id].parent_id;
        }
        return path_length;
    }

    int RRTPlanner::getNearestNodeId(const double *point) {  //MOD

        int nearest_node_id = -1;
        double min_dist = std::numeric_limits<double>::max();

        // Iterate through the existing nodes to find the nearest one
        for(size_t i = 0; i < nodes_.size(); ++i){
            double dist = computeDistance(nodes_[i].pos, point);

            if(dist < min_dist){
                min_dist = dist;
                nearest_node_id = i;
            }
        }

        return nearest_node_id;

    }

    void RRTPlanner::createNewNode(const double* pos, int parent_node_id) { //MOD

        Node new_node;

        // set the position of the new node
        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];

        // set the parent node ID
        new_node.parent_id = parent_node_id;

        //set the node ID as the next available ID in the nodes vector
        new_node.node_id = nodes_.size();

        //add the new node to the tree
        nodes_.emplace_back(new_node);
        
    }

    double* RRTPlanner::sampleRandomPoint() { //MOD
        //generate a random prob in the range [0,1]
        double random_prob = random_double_x.generate();

        //prob of sampling near the goal
        double near_goal_probability = 0.2;

        if(random_prob < near_goal_probability){
            //vector from the robot to the goal
            double vec_to_goal_x = goal_[0] - start_[0];
            double vec_to_goal_y = goal_[1] - start_[1];

            //calculate the angle
            double angle_to_goal = atan2(vec_to_goal_y, vec_to_goal_x);

            double random_angle_offset = (M_PI / 2) * (2 * random_double_x.generate() - 1);

            double random_angle = angle_to_goal + random_angle_offset;

            double max_distance = 0.1;
            double random_distance = max_distance * random_double_x.generate();

            rand_point_[0] = start_[0] + random_distance * cos(random_angle);
            rand_point_[1] = start_[1] + random_distance * sin(random_angle);

        } else {
            rand_point_[0] = random_double_x.generate();
            rand_point_[1] = random_double_y.generate();
        }

        return rand_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) { //MOD

        // Compute the distance between nearest and rand
        double distance = computeDistance(point_nearest, point_rand);
        double direction_x = point_rand[0] - point_nearest[0];
        double direction_y = point_rand[1] - point_nearest[1];

        // If dist is less than the value of step then candidate point is the random point
        if(distance <= params_.step){
            candidate_point_[0] = point_rand[0];
            candidate_point_[1] = point_rand[1];
        } else {
            candidate_point_[0] = point_nearest[0] + params_.step * (point_rand[0] - point_nearest[0]) / distance;
            candidate_point_[1] = point_nearest[1] + params_.step * (point_rand[1] - point_nearest[1]) / distance;
        }

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
        if(goalBuf_[0] != goal[0] || goalBuf_[1] != goal[1]){
            prev_path_length = 0;
            std::cout << "check" << std::endl;
        }
        goal_[0] = goal[0];
        goal_[1] = goal[1];

        goalBuf_[0] = goal[0];
        goalBuf_[1] = goal[1];
    }

};