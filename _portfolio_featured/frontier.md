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
The goal of this project was to create an easy to use platform to assess the performance of robot exploration in various environments with both full and limited field of view sensor data. Put simply, the question is "if I change the way that my robot moves, how does that affect it's ability to explore?". 

### Scope
To make this approachable and easy to use, I chose to use common platforms and packages that are widely used in the robotics community. In this specific case, I used `slam_toolbox` for map generation, the Nav2 stack for path planning and execution, and differential drive robots like the Clearpath Jackal and Turtlebot3 for testing. I have written 5 methods of frontier exploration to exhibit and compare different behaviors in the robot. Details on each are given below

### Output
All of my code is written as ROS2 packages that can be downloaded from GitHub and built from source. All code is written in C++ to minimize runtime delay and allow for more platform flexibility. Through the readme, any user should be able to build the packages and then use the lauch files to either run a full simulation or deploy frontier exploration on a real robot. 

### Source code
[Github Repository](https://github.com/Schelbert197/Frontier_toolbox_ROS2)

## Main Hardware Components
  - Clearpath Jackal Robot running ROS2 Humble on Ubuntu 22.04
  - Velodyne VLP16 3D lidar
  - Custom Hitch Mechanism 
  - Linksys E8450 Router with OpenWRT firmware 23.05
  - User Laptop (also running ROS2 Humble on Ubuntu 22.04)
  - AGRI-FAB Tow Broadcast Spreader Trailer

  ![saltbot]({{ site.url }}{{ site.baseurl }}/assets/images/Saltbot.png)

## Main Software Components
  - slam_toolbox
  - Nav2 Stack
  - Robot setup (`robot_control`) package
  - frontier exploration (`frontier_exp_cpp`) package
  - Nav2 action client (`nav_client_cpp`) package

### Frontier Node Decision Making Process
![saltbot]({{ site.url }}{{ site.baseurl }}/assets/images/final_project/Map_processing2.drawio.png)

### Creating Waypoints
If a map is present, the user may run the service call `ros2 service call lean_waypoints std_srvs/srv/Empty` which will generate waypoints that the robot can follow. This command involves a few steps:
1. Naively place a waypoint of the specified cell radius anywhere within an open space that meets the criteria.
2. Run through each waypoint generated, and remove waypoints that are islands or peninsulas (only having 0 or 1 neighbor) to eliminate any waypoints that are unreachable or could trap the robot.
- Since the robot has the trailer, the assumption is that it may not back up.
3. Add an orientation to each waypoint for the robot so that it sets up a heading for the next waypoint.
4. Save the waypoints and publish them as a set of markers to be visualized in Rviz.

A purple puck marker indicates the area of a waypoint cell, and the green arrow shows the orientation or heading for the robot. The image below shows a map created in rviz from a Gazebo simulation where the pucks have been placed in the confidently open spaces and the arrows show the direction of travel for the robot to take a "lawnmower" like path.

![sim_path]({{ site.url }}{{ site.baseurl }}/assets/images/sim_path2.png)

### Traveling
Once the plan has been written, the user can call the service `ros2 service call travel std_srvs/srv/Empty` that initiates the robot. When the service is called, the state machine will leave the "IDLE" state and switch to the "SEND GOAL" state where the robot will plan a path to the first waypoint. The "SEND GOAL" state interfaces with the nav_node which calls the nav to waypoint action client. The planner then waits in the "AWAITING GOAL COMPLETION" state and echoes the current goal. After some time, the nav_node will respond with either "Succeeded", "Aborted", "Canceled" or "Unknown Error" messages which the planner state machine can triage. 

If the message reads "Succeeded", the planner state machine then switches to the "SEND GOAL" state again and sends the next waypoint to nav_node to plan with Nav2. The image below shows the red path that the robot will take to the next waypoint accounting for the obstacle inflation cost. If there are no more waypoints from the original plan, then the state machine returns to "IDLE" with a message indicating to the user that the route was successfully completed. 

If the message reads "Aborted", the planner state machine will check a flag deciding to reset upon abort. If the flag reads true, the state machine returns to "IDLE" and removes any progress of the overall plan. If the flag reads false, the state machine will alert the user that it is skipping that waypoint in the plan and is looking to move to the next waypoint in the plan. It will switch to the "SEND GOAL" state, send the pose of the waypoint at the next index, and then await completion of the next move. 

In the case that the response is either "Canceled" or "Unknown Error", the state machine simply clears the progress and resets to idle. The user can replan at any time. If the robot is misbehaving or the mission needs to be cancelled, the user can simply run the service `ros2 service call saltbot_cancel_nav std_srvs/srv/Empty` which will cancel a movement to a waypoint cleanly (rather than killing the nav node or using the E-stop button).

![local_nav]({{ site.url }}{{ site.baseurl }}/assets/images/local_nav2.png)

## Nav Node
The nav node is made to handle the interfacing with the Nav2 stack. It has it's own state machine and allows the planner to remain separate from the Nav2 interface. 

### Nav to Pose
When the planner sends a pose, the nav node takes in the x, y, and yaw of the pose since the robot is considered to be moving in a 2D map. Like the planner, the state switches from "IDLE" to "SEND GOAL" where the NavigateToPose action client for Nav2 is called. In the "SEND GOAL" state, the nav node attempts to contact the action server. If the server is inactive or the Nav2 node died, then the nav node will log an error and abort the waypoint (which will also alert the planner). 

If the action call succeeds, then the state will switch to "WAIT FOR GOAL RESPONSE". If this state gets a message that the goal was rejected by the Nav2 server, the node alerts the user and returns to idle. This could happen due to no feasible paths being available. If the local plan is generated by Nav2, then the state moves to "WAIT FOR MOVEMENT COMPLETE" where the feedback callback will print the current pose of the robot until the goal is completed. If the robot is unable to move for a specified amount of time (in this case 30 seconds) the movement will be considered aborted.

Once feedback on a goal status is achieved (success, abort, etc.), then the node will respond to the planner. 

### Indoor Planning Video Demo
The video below shows the RViz screen as the robot moves through some of the middle waypoints sent from the planner. The red path shows the local plan it creates as it navigates from point to point. The dark parts of the map are occupied (obstacles) and the colored "shorelines" surrounding the dark areas is the high cost area that the robot attempts to avoid during it's planning. 

<iframe width="480" height="400" src="https://www.youtube.com/embed/z3MtAmL7pZU" title="Saltbot indoor planning" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

## Future work
With more time, I would like to add the following functionalities to make the saltbot more robust:
1. Adding a GPS enabled waypoint capability, where the robot can salt sidewalks and pathways using a GPS RTK (real-time kinematics) positioning system.
2. Allowing the path planning algorithm to allow for "one-way loops" that gives the robot the ability to cover more patio space even in tighter spots.
  - Currently, since the algorithm does not allow a peninsula, this is not likely to be planned.

## Outdoor
### Video Demo
<iframe width="560" height="315" src="https://www.youtube.com/embed/Ds8fOuWjql8?si=h1Npc1BC5I5yZ3La" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The above video demonstrates the robot traveling with the salt trailer it uses for this project.


## Source code
[Github Repository](https://github.com/Schelbert197/saltbot.git)