
#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {

        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();

    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) { //MOD

        // convert world coordiantes to map grid coordinates
        unsigned int mx, my;
        if(!costmap_->worldToMap(world_pos[0], world_pos[1], mx, my)){
            // If the point is outside the map boundaries, it's not free space
            return false;
        }

        //get the cost value at the map cell
        unsigned char cost = costmap_->getCost(mx,my);

        return cost == costmap_2d::FREE_SPACE;

    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {

        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return ( !inFreeSpace(point_b) ) ? true : false;

        } else {
            
            int num_steps = static_cast<int>(floor(dist/resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {

                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }

    }

};