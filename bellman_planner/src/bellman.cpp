#include <pluginlib/class_list_macros.h>
#include "../include/custom_bellman_planner/bellman.h"
#include <tf2/utils.h>
#include <angles/angles.h>
#include <cmath>
#include <algorithm>

namespace custom_bellman_planner {

PLUGINLIB_EXPORT_CLASS(custom_bellman_planner::BellmanFord, nav_core::BaseLocalPlanner)

BellmanFord::BellmanFord() : costmap_ros_(nullptr), tf_(nullptr), initialized_(false), goal_reached_(false) {
    global_plan_ = std::vector<geometry_msgs::PoseStamped>();
}

BellmanFord::BellmanFord(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros,
                         bool verbose)
    : costmap_ros_(nullptr), tf_(nullptr), initialized_(false), goal_reached_(false), verbose_(verbose) {
    global_plan_ = std::vector<geometry_msgs::PoseStamped>();
    initialize(name, tf, costmap_ros);
}

BellmanFord::~BellmanFord() {}

void BellmanFord::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        costmap_ros_ = costmap_ros;
        tf_ = tf;
        initialized_ = true;
        if (verbose_) {
            ROS_INFO("Initialization complete");
        }
    } else {
        if (verbose_) {
            ROS_WARN("Already initialized");
        }
    }
}

bool BellmanFord::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        if (verbose_) {
            ROS_ERROR("This planner has not been initialized");
        }
        return false;
    }
    global_plan_ = plan;
    goal_reached_ = false;
    return true;
}

bool BellmanFord::transformGlobalPlan(const costmap_2d::Costmap2DROS& costmap_ros,
                                      const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                      const geometry_msgs::PoseStamped& global_pose,
                                      std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
    transformed_plan.clear();
    geometry_msgs::TransformStamped tf_pose;
    try {
        tf_pose = tf_->lookupTransform(costmap_ros.getBaseFrameID(), global_pose.header.frame_id, ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("FAILED TO TRANSFORM GLOBAL PLAN: %s", ex.what());
        return false;
    }

    for (const auto& pose : global_plan) {
        geometry_msgs::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, tf_pose);
        transformed_plan.push_back(transformed_pose);
    }
    return true;
}

bool BellmanFord::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }
    if (global_plan_.empty()) {
        if (verbose_) {
            ROS_WARN("Empty plan");
        }
        return false;
    }

    // Get the current robot pose
    geometry_msgs::PoseStamped current_pose;
    try {
        if (!costmap_ros_->getRobotPose(current_pose)) {
            ROS_ERROR("Couldn't get robot pose");
            return false;
        }
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        return false;
    }

    // Transform the global plan to the local frame
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!transformGlobalPlan(*costmap_ros_, global_plan_, current_pose, transformed_plan)) {
        if (verbose_) {
            ROS_WARN("Couldn't transform global plan to local frame");
        }
        return false;
    }

    // Apply Bellman-Ford algorithm to find the shortest path
    std::vector<int> distances(transformed_plan.size(), std::numeric_limits<int>::max());
    std::vector<int> predecessors(transformed_plan.size(), -1);
    distances[0] = 0;

    for (size_t i = 0; i < transformed_plan.size() - 1; ++i) {
        for (size_t u = 0; u < transformed_plan.size(); ++u) {
            for (size_t v = 0; v < transformed_plan.size(); ++v) {
                if (u != v) {
                    double dx = transformed_plan[v].pose.position.x - transformed_plan[u].pose.position.x;
                    double dy = transformed_plan[v].pose.position.y - transformed_plan[u].pose.position.y;
                    double distance = std::sqrt(dx * dx + dy * dy);
                    if (distances[u] + distance < distances[v]) {
                        distances[v] = distances[u] + distance;
                        predecessors[v] = u;
                    }
                }
            }
        }
    }

    // Extract the shortest path from the Bellman-Ford results
    std::vector<geometry_msgs::PoseStamped> shortest_path;
    for (int v = transformed_plan.size() - 1; v != -1; v = predecessors[v]) {
        shortest_path.push_back(transformed_plan[v]);
    }
    std::reverse(shortest_path.begin(), shortest_path.end());

    if (shortest_path.empty()) {
        if (verbose_) {
            ROS_WARN("The shortest path is empty, cannot compute velocity commands");
        }
        return false;
    }

    // Generate velocity commands to follow the shortest path
    geometry_msgs::PoseStamped target_pose = shortest_path[1];
    double target_x = target_pose.pose.position.x;
    double target_y = target_pose.pose.position.y;

    double error_x = target_x - current_pose.pose.position.x;
    double error_y = target_y - current_pose.pose.position.y;
    double distance_to_target = std::sqrt(error_x * error_x + error_y * error_y);

    double angle_to_target = std::atan2(error_y, error_x);
    double yaw = tf2::getYaw(current_pose.pose.orientation);
    double angle_error = angles::shortest_angular_distance(yaw, angle_to_target);

    // Set velocity commands
    cmd_vel.linear.x = std::min(0.5, distance_to_target);
    cmd_vel.angular.z = std::min(0.5, angle_error);

    // Check if the goal is reached
    if (distance_to_target < 0.1 && std::fabs(angle_error) < 0.1) {
        goal_reached_ = true;
    } else {
        goal_reached_ = false;
    }

    return true;
}

bool BellmanFord::isGoalReached() {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized");
        return false;
    }

    return goal_reached_;
}

}

