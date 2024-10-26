
#include <rrt_planner/rrt_planner.h>
#include <pluginlib/class_list_macros.h>
#include <graph_msgs/GeometryGraph.h>
#include <graph_msgs/Edges.h>
#include <sensor_msgs/PointCloud.h>

#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <ros/ros.h>
#include <limits>
#include <cmath>
#include <fstream>

#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params, ros::Publisher &nodes_publisher_, ros::Publisher &edges_publisher_) : params_(params), collision_dect_(costmap), nodes_publisher_(nodes_publisher_), edges_publisher_(edges_publisher_) {

        costmap_ = costmap->getCostmap();
        node_num = 0;
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);

        num_paths_calculated = 0;
        num_paths_calculated_success = 0;
        num_nodes_blocked = 0;
        total_path_calc_time = 0;
        new_goal=false;
        num_nodes = 0;
        num_iterations = 0;

        last_amcl_pose_[0] = std::numeric_limits<double>::quiet_NaN();
        last_amcl_pose_[1] = std::numeric_limits<double>::quiet_NaN();
        amcl_distance_traveled = 0.0;
        amcl_sub_ = nh_.subscribe("/amcl_pose", 10, &RRTPlanner::amclCallback, this);

        log_file_.open("/home/ir-labs/catkin_ws/src/rrt_planner_git/src/rrt_planer_metrics.csv", std::ios::out | std::ios::app);

        if (log_file_.tellp() == 0){
            log_file_ << "NumPathsCalculated, NumIterations, NumPathsSuccess, NumPathsBlocked, PathCalcTime, NumNodes, NodesUsedInPath, IsNewGoal, RealDistance, AmclDistance, RRTDistance\n";
        }
    }

    bool RRTPlanner::planPath() {

	if(goal_[0] != old_goal_[0] || goal_[0] != old_goal_[0] ){
		new_goal = true;
		old_goal_[0] = goal_[0];
		old_goal_[1] = goal_[1];
	}else{
		new_goal = false;
	}
        start_time = ros::Time::now();

        // clear everything before planning
        node_num = 0;
        nodes_.clear();
        num_nodes = 0;
        num_nodes_blocked = 0;
        num_iterations = 0;
        last_amcl_pose_[0] = std::numeric_limits<double>::quiet_NaN();
        last_amcl_pose_[1] = std::numeric_limits<double>::quiet_NaN();

        // Start Node
        createNewNode(start_, -1);

        double *p_rand, *p_new;
        Node nearest_node;
        Node last_node;
        bool success = false;

        for (unsigned int k = 1; k <= params_.max_num_nodes;) {
            num_iterations++;
            p_rand = sampleRandomPoint();
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                last_node = createNewNode(p_new, nearest_node.node_id);
                k++;

            } else {
                num_nodes_blocked++;
                continue;
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance) {
                    success = true;
                    num_paths_calculated_success++;
                    break;
                }
            }
        }
        ros::Duration calc_time = ros::Time::now() - start_time;
        total_path_calc_time += calc_time.toSec();
        num_paths_calculated++;
        logMetrics(last_node); 
        return success;
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

    Node RRTPlanner::createNewNode(const double* pos, int parent_node_id) { //MOD

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

        publishTree();

        num_nodes++;
        return new_node;        
    }

    void RRTPlanner::publishTree() {

        sensor_msgs::PointCloud cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = ros::Time::now();

        visualization_msgs::Marker edges;
        edges.header.frame_id = "map";
        edges.ns = "tree_edges";
        edges.type = visualization_msgs::Marker::LINE_LIST;
        edges.action = visualization_msgs::Marker::ADD;
        edges.pose.orientation.w = 1.0;
        edges.scale.x = 0.02;
        edges.color.r = 1.0;
        edges.color.a = 1.0;

        for (Node node : nodes_) {
            geometry_msgs::Point32 p;
            p.x = node.pos[0];
            p.y = node.pos[1];
            p.z = 0.0;
            cloud.points.push_back(p);

            if (node.parent_id != -1) {

                geometry_msgs::Point start, end;

                start.x = nodes_[node.parent_id].pos[0];
                start.y = nodes_[node.parent_id].pos[1];
                start.z = 0.0;

                end.x = node.pos[0];
                end.y = node.pos[1];
                end.z = 0.0;

                edges.points.push_back(start);
                edges.points.push_back(end);
            }
        }   

        nodes_publisher_.publish(cloud);
        edges_publisher_.publish(edges);  
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

    void RRTPlanner::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        
        if(!std::isnan(last_amcl_pose_[0]) || !std::isnan(last_amcl_pose_[1])){
            double current_amcl_pose[2] = {x, y};
            amcl_distance_traveled += computeDistance(current_amcl_pose, last_amcl_pose_);
        }

        last_amcl_pose_[0] = x;
        last_amcl_pose_[1] = y;
    }

    void RRTPlanner::logMetrics(Node node){
        ROS_INFO("Log Metrics (%d)", num_paths_calculated);
        Node next_node = node;
        int count = 1;
        double rrt_distance = 0;
        double real_distance = computeDistance(start_, goal_);
        while(next_node.parent_id > 0 || next_node.parent_id == -1){
            if(next_node.parent_id == -1) {
                rrt_distance += computeDistance(next_node.pos, start_);
                break;
            }
            count++;
            Node parent_node = nodes_[next_node.parent_id];
            rrt_distance += computeDistance(next_node.pos, parent_node.pos);
            next_node = parent_node;
        }
        if(log_file_.is_open()){
            log_file_ << num_paths_calculated << "," << num_iterations  << "," << num_paths_calculated_success  << "," << num_nodes_blocked  << "," << total_path_calc_time  << "," << num_nodes  << "," << count  << "," << new_goal << "," << real_distance << "," << amcl_distance_traveled << "," << rrt_distance << "\n";
        }else{
            std::cerr << "unable to open log file for writng." << std::endl;
        }
        amcl_distance_traveled = 0.0;
    }       

    RRTPlanner::~RRTPlanner(){
        if(log_file_.is_open()){
            log_file_.close();
        }
    }

};
