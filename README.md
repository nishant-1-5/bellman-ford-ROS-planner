# Custom Bellman-Ford Planner for Navigation

## Note
This repository uses Ros-Noetic and Gazebo 11.11.0

## Basic Information
This repository implements a custom local planner for the ROS navigation stack using the Bellman-Ford algorithm. The planner calculates the shortest path from the robot's current position to a target goal within a known map. This repo includes the necessary source code, param files, and launch files to integrate the planner into a TurtleBot3 simulation in Gazebo. The instructions are written such that you are continuing from the previous repo

## Installation
Clone this repo in your workspace containing your nav-stack.
```
git clone https://github.com/nishant-1-5/Autonomous-navigation-using-gazebo-and-ROS.git
cd ~/catkin_ws/
catkin_make
```
Make sure to source the workspace.
```
source devel/setup.bash
```
Navigate to your the params directory of the navigation package, for tutrtlebot3 it may look something like this:-
```
src/turtlebot3/turtlebot3_navigation/params
```
Now in local_costmap_params.yaml add the following line:-
```
base_local_planner: "custom_bellman_planner/BellmanFord"
```
Now create a new param file in this directory named bellman_ford_planner_params.yaml and copy the following code to it:-
```
BellmanFord:
	verbose: true
```
Now edit the move_base.launch file to accommodate it
Replace 
```
<rosparam file="$(find turtlebot3_navigation)/param/dwa_local_planner_params_$(arg model).yaml" command="load" />
```
with 
```    
<rosparam file="$(find turtlebot3_navigation)/param/bellman_ford_planner_params.yaml" command="load" />
```

# TODO
Cant manage to fix transforms after 2 days of trying

