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
<rosparam file="$(find bellman_planner)/param/bellman_ford_planner_params.yaml" command="load" />
```
and so on..

map file loc is specified by default in launch file 
map made with [This repo](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin).

## BELLMAN-FORD-Algorithm
It is a dynamic-programming based approach to find the shortest path in a graph, main use case of Bellman-Ford algorithm is when the graph contains negative weights where dijktras algorithm has undefined behaviour. It is also used to find if the graph contains cyle of Edges whose sum of weights in negative i.e. a negative weight cycle.

The algoriths is as follows:-

### Bellman-Ford(E, V, s):

#### Input:

    E: List of edges in the graph, where each edge is represented as a tuple (u, v, w) indicating an edge from vertex uu to vertex vv with weight ww.
    V: List of vertices in the graph.
    s: The source vertex from which shortest paths are to be calculated.

#### Output:

    Distance array dist[] where dist[v] is the shortest distance from source s to vertex v.
    Predecessor array pred[] to reconstruct the shortest paths.
    A boolean indicating if a negative weight cycle exists.
#### Pseudo-Code and Explanation
```
function Bellman-Ford(E, V, s):
    // Step 1: Initialize distances from source to all vertices as infinite
    // and distance to the source itself as 0
    dist[] = array of size |V| with all values set to INF
    pred[] = array of size |V| with all values set to NIL
    dist[s] = 0
    
    // Step 2: Relax all edges |V|-1 times
    for i from 1 to |V|-1:
        for each edge (u, v, w) in E: // i.e u != v
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                pred[v] = u

    // Step 3: Check for negative weight cycles
    for each edge (u, v, w) in E:
        if dist[u] + w < dist[v]:
            // Negative weight cycle found
            return (dist, pred, true)
    
    // No negative weight cycle found
    return (dist, pred, false)
```
#### Time Complexity
This algorithm has a time complexity O(|V||E|) i.e in worst case when |E| = |v|^2 , it is O(n^3)


After
# TODO
Cant manage to fix transforms after 2 days of trying

