
#include <rrt_planner/rrt_planner.h>
#include <pluginlib/class_list_macros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <sensor_msgs/PointCloud.h>

#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params, ros::Publisher &vertices_pub, ros::Publisher &edges_pub) : params_(params), collision_dect_(costmap), vertices_pub_(vertices_pub), edges_pub_(edges_pub) {

        costmap_ = costmap->getCostmap();
        node_num = 0;
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        node_num = 0;
        nodes_.clear();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;

        for (unsigned int k = 1; k <= params_.max_num_nodes;) {

            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                createNewNode(p_new, nearest_node.node_id);
                k++;

            } else {
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance) {
                    return true;
                }
            }
        }

        return false;
    }

    // not used
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
                nearest_node_id = nodes_[i].node_id;
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
        new_node.node_id = node_num++;

        //add the new node to the tree
        nodes_.push_back(new_node);

        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = ros::Time::now();

        for (Node n : nodes_)
        {
            geometry_msgs::Point32 p;
            p.x = n.pos[0];
            p.y = n.pos[1];
            p.z = 0.0;
            cloud.points.push_back(p);
        }

        vertices_pub_.publish(cloud);

        visualization_msgs::Marker edges;
        edges.header.frame_id = "map";
        edges.ns = "tree_edges";
        edges.type = visualization_msgs::Marker::LINE_LIST;
        edges.action = visualization_msgs::Marker::ADD;

        edges.pose.orientation.w = 1.0;
        edges.scale.x = 0.01;

        edges.color.r = 1.0;
        edges.color.a = 1.0;

        for (const auto &node : nodes_)
        {
            if (node.parent_id == -1)
                continue;

            geometry_msgs::Point start, end;

            const auto &parent = nodes_[node.parent_id];

            start.x = parent.pos[0];
            start.y = parent.pos[1];
            start.z = 0.0;

            end.x = node.pos[0];
            end.y = node.pos[1];
            end.z = 0.0;

            edges.points.push_back(start);
            edges.points.push_back(end);
        }

        edges_pub_.publish(edges);         
    }

    double* RRTPlanner::sampleRandomPoint() { //MOD
        double random_prob = random_double_x.generate();
        double near_goal_probability = 0.6;

        double *random_point_ = new double[2];

        if(random_prob < near_goal_probability){
            double vec_to_goal_x = goal_[0] - start_[0];
            double vec_to_goal_y = goal_[1] - start_[1];

            double angle_to_goal = atan2(vec_to_goal_y, vec_to_goal_x);

            double cone_angle = M_PI / 6;
            double random_angle_offset = cone_angle * (2 * random_double_x.generate() - 1);

            double random_angle = angle_to_goal + random_angle_offset;

            double max_distance = 0.3;
            double random_distance = max_distance * random_double_x.generate();

            random_point_[0] = start_[0] + random_distance * cos(random_angle);
            random_point_[1] = start_[1] + random_distance * sin(random_angle);
        } else {
            random_point_[0] = random_double_x.generate();
            random_point_[1] = random_double_y.generate();
        }

        return random_point_;
    }

    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) { //MOD

        // Compute the distance between nearest and rand
        double distance = computeDistance(point_nearest, point_rand);

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
        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

};