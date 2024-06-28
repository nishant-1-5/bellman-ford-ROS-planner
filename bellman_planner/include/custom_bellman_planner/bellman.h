#ifndef _BELLMAN_H
#define _BELLMAN_H

#include<ros/ros.h>
#include<nav_core/base_local_planner.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<costmap_2d/costmap_2d.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Path.h>
#include<tf2_ros/buffer.h>
#include<string>
#include<vector>

namespace custom_bellman_planner{

class BellmanFord: public nav_core::BaseLocalPlanner{
public:
	BellmanFord();
	~BellmanFord();
        BellmanFord(std::string name,tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS *costmap_ros,
		    bool verbose=true);
        void initialize(std::string name, tf2_ros::Buffer* tf ,costmap_2d::Costmap2DROS *costmap_ros) override;
	
	bool transformGlobalPlan(const costmap_2d::Costmap2DROS& costmap_ros, 
			const std::vector<geometry_msgs::PoseStamped>& global_plan,
		       	const geometry_msgs::PoseStamped& global_pose, 
			std::vector<geometry_msgs::PoseStamped>& tranformed_plan
			);

	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
	bool isGoalReached()override;
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
                private:
                        costmap_2d::Costmap2DROS* costmap_ros_;
			tf2_ros::Buffer* tf_;
			std::vector<geometry_msgs::PoseStamped> global_plan_;
			bool goal_reached_;
                        bool initialized_;
			bool verbose_;
        };
}


#endif
