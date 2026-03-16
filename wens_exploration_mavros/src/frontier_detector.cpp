/**
 * @file frontier_detector.cpp
 * @brief 3D Frontier-based exploration detector for WENS UAV.
 *
 * This node processes the OctoMap to detect exploration frontiers
 * (boundaries between known free space and unknown space) and
 * publishes the best frontier goal for the UAV to navigate to.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <cmath>

struct Frontier {
    double x, y, z;
    double score;
};

class FrontierDetector {
public:
    FrontierDetector() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Parameters
        pnh.param("exploration_altitude", exploration_altitude_, 2.0);
        pnh.param("min_frontier_size", min_frontier_size_, 5);
        pnh.param("publish_rate", publish_rate_, 1.0);
        pnh.param("uav_frame", uav_frame_, std::string("base_link"));
        pnh.param("map_frame", map_frame_, std::string("map"));

        // Subscribers
        octomap_sub_ = nh.subscribe<octomap_msgs::Octomap>(
            "octomap_binary", 1, &FrontierDetector::octomapCallback, this);
        uav_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
            "mavros/local_position/pose", 10, &FrontierDetector::poseCallback, this);

        // Publishers
        frontier_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
            "exploration/frontier_goal", 10);
        frontier_viz_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
            "exploration/frontiers_viz", 10);

        has_map_ = false;
        has_pose_ = false;

        ROS_INFO("[FrontierDetector] Initialized. Exploration altitude: %.1f m",
                 exploration_altitude_);

        timer_ = nh.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &FrontierDetector::timerCallback, this);
    }

private:
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
        if (tree) {
            octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
            has_map_ = (octree_ != nullptr);
        }
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        uav_pose_ = *msg;
        has_pose_ = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!has_map_ || !has_pose_) return;

        std::vector<Frontier> frontiers = detectFrontiers();
        if (frontiers.empty()) {
            ROS_WARN_THROTTLE(10, "[FrontierDetector] No frontiers found.");
            return;
        }

        // Select best frontier (highest score)
        auto best = std::max_element(frontiers.begin(), frontiers.end(),
            [](const Frontier& a, const Frontier& b) { return a.score < b.score; });

        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = map_frame_;
        goal.pose.position.x = best->x;
        goal.pose.position.y = best->y;
        goal.pose.position.z = exploration_altitude_;
        goal.pose.orientation.w = 1.0;

        frontier_pub_.publish(goal);
        publishFrontierViz(frontiers, *best);

        ROS_INFO("[FrontierDetector] Best frontier: (%.2f, %.2f) score=%.2f, total=%zu",
                 best->x, best->y, best->score, frontiers.size());
    }

    std::vector<Frontier> detectFrontiers() {
        std::vector<Frontier> frontiers;
        if (!octree_) return frontiers;

        double uav_x = uav_pose_.pose.position.x;
        double uav_y = uav_pose_.pose.position.y;
        double uav_z = uav_pose_.pose.position.z;
        double res = octree_->getResolution();

        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(),
             end = octree_->end_leafs(); it != end; ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();

                // Check if this free cell has unknown neighbors (frontier condition)
                int unknown_count = 0;
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) continue;
                            octomap::point3d neighbor(x + dx * res, y + dy * res, z + dz * res);
                            octomap::OcTreeNode* node = octree_->search(neighbor);
                            if (node == nullptr) unknown_count++;
                        }
                    }
                }

                if (unknown_count >= min_frontier_size_) {
                    Frontier f;
                    f.x = x;
                    f.y = y;
                    f.z = z;
                    // Score: prefer frontiers that are far from current position but accessible
                    double dist = std::sqrt(std::pow(x - uav_x, 2) + std::pow(y - uav_y, 2));
                    f.score = static_cast<double>(unknown_count) * std::log(1.0 + dist);
                    frontiers.push_back(f);
                }
            }
        }

        return frontiers;
    }

    void publishFrontierViz(const std::vector<Frontier>& frontiers, const Frontier& best) {
        visualization_msgs::MarkerArray markers;

        // All frontiers as small spheres
        visualization_msgs::Marker all_marker;
        all_marker.header.frame_id = map_frame_;
        all_marker.header.stamp = ros::Time::now();
        all_marker.ns = "frontiers";
        all_marker.id = 0;
        all_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        all_marker.action = visualization_msgs::Marker::ADD;
        all_marker.scale.x = 0.2;
        all_marker.scale.y = 0.2;
        all_marker.scale.z = 0.2;
        all_marker.color.r = 0.0;
        all_marker.color.g = 0.7;
        all_marker.color.b = 1.0;
        all_marker.color.a = 0.7;

        for (const auto& f : frontiers) {
            geometry_msgs::Point p;
            p.x = f.x; p.y = f.y; p.z = exploration_altitude_;
            all_marker.points.push_back(p);
        }
        markers.markers.push_back(all_marker);

        // Best frontier as large sphere
        visualization_msgs::Marker best_marker;
        best_marker.header.frame_id = map_frame_;
        best_marker.header.stamp = ros::Time::now();
        best_marker.ns = "best_frontier";
        best_marker.id = 1;
        best_marker.type = visualization_msgs::Marker::SPHERE;
        best_marker.action = visualization_msgs::Marker::ADD;
        best_marker.pose.position.x = best.x;
        best_marker.pose.position.y = best.y;
        best_marker.pose.position.z = exploration_altitude_;
        best_marker.pose.orientation.w = 1.0;
        best_marker.scale.x = 0.5;
        best_marker.scale.y = 0.5;
        best_marker.scale.z = 0.5;
        best_marker.color.r = 1.0;
        best_marker.color.g = 0.3;
        best_marker.color.b = 0.0;
        best_marker.color.a = 1.0;
        markers.markers.push_back(best_marker);

        frontier_viz_pub_.publish(markers);
    }

    // ROS handles
    ros::Subscriber octomap_sub_, uav_pose_sub_;
    ros::Publisher frontier_pub_, frontier_viz_pub_;
    ros::Timer timer_;

    // State
    std::shared_ptr<octomap::OcTree> octree_;
    geometry_msgs::PoseStamped uav_pose_;
    bool has_map_, has_pose_;

    // Parameters
    double exploration_altitude_;
    int min_frontier_size_;
    double publish_rate_;
    std::string uav_frame_, map_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontier_detector");
    FrontierDetector detector;
    ros::spin();
    return 0;
}
