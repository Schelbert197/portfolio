---
title: "Frontier Toolbox: Autonomous Exploration"
author_profile: true
key: 1
toc: true
excerpt: "Frontier Exploration, SLAM, Navigation, ROS2, Nav2"
header:
  teaser: /assets/images/final_project/crb_frontier1.png
classes: wide
---
## Featured Video
<iframe width="560" height="315" src="https://www.youtube.com/embed/tb6XYFM1r3c?si=1y1yYJZhaTsBZS6s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Project Overview
### Objective
The goal of this project was to create an easy to use platform to assess the performance of robot frontier exploration in various environments with both full and limited field of view sensor data. Put simply, the question is "if I change the way that my robot moves, how does that affect it's ability to explore?". 

### Scope
To make this approachable and easy to use, I chose to use common platforms and packages that are widely used in the robotics community. In this specific case, I used `slam_toolbox` for map generation, the Nav2 stack for path planning and execution, and differential drive robots like the Clearpath Jackal and Turtlebot3 for testing. I have written 5 methods of frontier exploration to exhibit and compare different behaviors in the robot. Details on each are given below

### Output
All of my code is written as ROS2 packages that can be downloaded from GitHub and built from source. All code is written in C++ to minimize runtime delay and allow for more platform flexibility. Through the readme, any user should be able to build the packages and then use the lauch files to either run a full simulation or deploy frontier exploration on a real robot. 

### Source code
[Github Repository](https://github.com/Schelbert197/Frontier_toolbox_ROS2)

### Main Software Components
  - slam_toolbox
  - Nav2 Stack
  - Robot setup (`robot_control`) package
  - frontier exploration (`frontier_exp_cpp`) package
  - Nav2 action client (`nav_client_cpp`) package

## Robot Setup
The `robot_control` package includes several ROS2 nodes that allow the user to run slam_toolbox with some added features. The intercept node allows the user to dynamically change the FOV of the LaserScan message for study with low sensor data cases. By simply publisng with `ros2 topic pub /fov std_msgs/msg/Int64 "{data: <FOV_degrees>}"` the FOV of the LaserScan can be adjusted.
![limited_FOV_sim]({{ site.url }}{{ site.baseurl }}/assets/images/final_project/robot_example.png)
The pointcloud_to_laserscan node allows the user to easily convert 3D scans to 2D since slam_toolbox is a 2D LIDAR platform. This package also provides scripted paths for perfect repeatability in simulation.

## Interfacing With Nav2
The Nav2 stack is easy to use and can be utilized by simply publishing on the `/goal_pose` ROS2 topic, but for further control and and a closed feedback loop, I have created a package `nav_client_cpp` to handle action feedback. Unlike simply publishing on the `/goal_pose` topic, this allows for replanning upon failure, and diagnostic printouts. Through topics and service calls, this package can easily interface with a planner node/package.

## Frontier Navigation
The heart and soul of this project revolves around the frontier navigation package `frontier_exp_cpp` which exhibits 5 unique frontier exploration algorithms as well as an easy-to-use library of functions if a user would like to import the functions to calculate frontiers, clusters, or other values within their own nodes. When using my `frontier_lc` lifecycle node, a user can select the algorithm by adjusting parameters in the `frontier_params.yaml` file. The decision tree is shown below.

### Frontier Node Decision Making Process
![decision_diagram]({{ site.url }}{{ site.baseurl }}/assets/images/final_project/Map_processing2.drawio.png)

Based on this decision making process, this results in 5 unique algorithms:
1. Goal position is the closest single frontier to the robots "viewpoint" which is x[m] in front of the robot. [More about this algorithm]({{ site.url }}{{ site.baseurl }}/portfolio_featured/frontier/#distance-based-approach)
2. Goal position is the single frontier with the most information gain/entropy reduction if the robot were to be teleported there. [More about this calculation]({{ site.url }}{{ site.baseurl }}/portfolio_featured/frontier/#mutual-information-approach)
3. Goal position is the closest cluster centroid after the frontiers have been clustered with DBSCAN. [More about this algorithm]({{ site.url }}{{ site.baseurl }}/portfolio_featured/frontier/#distance-based-approach)
4. Goal position is the cluster centroid with the most information gain/entropy reduction if the robot were to be teleported there. [More about this calculation]({{ site.url }}{{ site.baseurl }}/portfolio_featured/frontier/#mutual-information-approach)
5. Goal position is the centroid of the cluster with the largest number of frontiers.

### About the Algorithms
Each algorithm performs differently which allows the robot to succeed in exploring various environments. Each one was judged based on a basis of learned map over time. 

#### Distance-based Approach
The initial "naive" approach was to have the robot simply select a frontier that is closest to its "viewpoint". The purpose of the viewpoint is to encourage the robot to keep moving forward in a greedy depth-first approach. Once the robot hits a dead end, it then selects the next closest frontier. If the robot get's stuck, this algorithm exploits the "spin" motion in Nav2's recovery behaviors to hopefully choose a different frontier and successfully create a path. 

#### Mutual Information Approach
Info and plots of info gain

![entropy_value]({{ site.url }}{{ site.baseurl }}/assets/images/final_project/entropy.png)

### Algorithm Comparisons
Comparison videos

### Results
Plots