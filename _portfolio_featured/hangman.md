---
title: "OCR Robot Arm Game Player"
author_profile: true
key: 5
toc: true
excerpt: "ROS2, MoveIt, OpenCV, Paddle OCR, Force Feedback Control"
header:
  teaser: /assets/images/playing_game.gif
classes: wide
gallery7892:
  - url: /assets/images/setting_up.gif
    image_path: /assets/images/setting_up.gif
    alt: "setting up"
    title: "Setting Up"
  - url: /assets/images/playing_game.gif
    image_path: /assets/images/playing_game.gif
    alt: "playing game"
    title: "Playing Game"
---

The aim of this project is to use the Franka Emika Panda 7-DoF arm to play the word game "hangman" with a human player in the loop.

## Group Members
This project is a group effort including Ananya Agarwal, Graham Clifford, Ishani Narwankar, Abhishek Sankar, and Srikanth Schelbert.

## Overview
This project is the culmination of 3 weeks of work showcasing the various skills and capabilities gained using ROS2 throughout the course. Our aim is that through playing hangman, our team can showcase robotic manipulation, Opitical Character Recognition (OCR), control theory, apriltag localization, and system design and integration.

### Short Video
<iframe width="560" height="315" src="https://www.youtube.com/embed/Q81Vcnj9kqs?si=1wjmTJCnAVbmtEMv" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The robot begins by initiating a "kickstart" sequence that localizes the board and draws the necessary lines on the board to make the game recognizable. This includes 5 dashes for the "secret" word, 5 dashes for incorrect guessed letters, and the location for the hangman when incorrect guesses are made. Once this sequence is complete, then the gameplay loop initiates until the game is complete. The rules of the game are as follows:
- The player shall win by correctly guessing the robot's chosen word before the whole stick figure is drawn.
- The robot will always choose a 5 letter word at random.
- The player may guess a letter or a 5 letter word at any point in the game.
- All incorrect guesses will be counted equally.
- The game ends either when the word is guessed (either letter-by-letter or all at once) or when the player has made 5 incorrect guesses.
{% include gallery id="gallery7892" %}

The steps of the robot's gameplay loop are as follows:
1. Move to face the player and initiate the OCR pipeline.
2. Use the OCR pipeline to read the entry and confirm the confidence is high enough to publish to the game player node.
  - The OCR pipline will turn off once the guess is published.
3. The player node processes the guess for correctness and evaluates what the robot will need to draw and where and publishes to the overall state machine.
4. The state machine processes these characters and positions into poses and trajectories and sends these trajectories to the robot until everything is drawn.
5. Upon completing the play, the robot then returns to facing the player described in step 1 unless the game has ended with a win or loss.

## Approach
This project was divided into a few concise subsystems:
- April Tag localization and calibration.
- Force Feedback control and motion planning.
- Gameplay and system state machine.
- CAD and gripper manipulation.
- OCR usage.

### Nodes
The image below shows the relationship between each node in the package. As the system integrator, my main tasks included writing the gameplay node and the state machine for the overal system. I also proposed the architecture of the package we created since my "brain" node is most central in the web and tracks the state of the system. To keep things concise and mitigate errors that were difficult to trace or failed silently, we sought to keep communication between nodes on a need basis. The diagram shows these communications, and arrows indicate one-way communication.
![Node_Diagram]({{ site.url }}{{ site.baseurl }}/assets/images/Node_diagram.png)

## OCR
The OCR pipline employed the use of the open-source PaddleOCR toolkit due to its speed and accuracy. Because of the need to process single letters and full words written by the player, we needed to ensure that the OCR was capable of reading both effectively. By processing two image feeds each optimized either for single letters or full words, the pipeline then creates a guess with high confidence that can be passed to the hangman node.

### OCR Videos
#### Single Letter
<iframe width="560" height="315" src="https://www.youtube.com/embed/AS6wPhCgIyg?si=ZpbtZHKk6cS10wyy" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
In this video, I am seen guessing the letter "R" which is correctly read by the OCR pipeline as shown in the terminal output. 

#### Full Word
<iframe width="560" height="315" src="https://www.youtube.com/embed/FbqclfQHlN0?si=pU_dTm06T0Ri8jYf" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
In this video, I am seen guessing a full word, in this case "FLOCK", which is again correctly read in the terminal output.

These show the high fidelity of a read allowing the player to trust that the robot sees and understands their guess.

## Custom MoveIt Library
In order for this project to be viable, a main task our team accomplished was writing a custom Python API for the robot to be able to move in space without issues with singularities. This library also includes custom functions to run the force on the end effector for the control. The following capabilities were granted with the implementation of our functions:
 - Calculating inverse kinematics to achieve the joint positions for a given pose.
 - Calculating the torques/forces on the end effector.
 - Queueing poses to create a trajectory.
 - Planning and executing both cartesian paths and MoveIt movements to given points.
 - Replanning paths that violate the force admittance control.

## April Tags
We used an AprilTag to localize the board. Since the known shape and size of the tag can convey position and orientation, the tag provides the robot with a transformation to the board through the tf tree. The "tags" node also provides the information regarding the location in which the robot draws the letters guessed by the player.

## Drawing Plans
For the robot to draw the letters, it needs to know what to draw, how to draw it, and where. To do this, the board was broken into a 6x8 grid of 10cm by 10cm tiles where each tile has a local origin as a reference for the drawn characters. The apriltag established the localization of these origin. Once the OCR pipeline sends a letter to the hangman player, it establishes the characters to be drawn along with positioning information. 

This information then gets processed by the "brain" node which takes the characters (given as a list of strings and tile coordinates) and generates a list of MoveIt poses and Cartesian trajectory paths using the "draw" node and our custom functions. These points along a path to draw a character are created in reference to the size of a tile so they are implemented in relation to that frame thanks to the "tags" node. The "brain" node also acts as the state machine for the whole system meaning it tracks what the robot is doing at any given time. The brain iterates through all of the service and action calls until everything for the turn has been written, and then returns to the viewing position as described in the overview.

## Full Gameplay (1x speed)
<iframe 
width="560" 
height="315" 
src="https://www.youtube.com/embed/tvCiYLAyh-0?si=ZKbWAICDMrywSt-V" 
title="YouTube video player" 
frameborder="0" 
allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" 
allowfullscreen></iframe>

### Video Recap
In the video, the initial kickstart sequence is seen drawing the dashes to setup the game as well as the location in which the hangman will be drawn. Shortly after, I am seen showing the robot the letter "Q" that acts as my guess for the turn. Since the letter is not in the word that the robot chose, my guess is incorrect. This means the letter I guessed is written in the lower portion of the board (where wrong guesses go) and the first portion of the stick figure (the head) is drawn. 

To demonstrate how the robot handles a full word (and a correct guess), I temporarily have the robot print the word it chose to the terminal. The word "fight", which was chosen by the robot at the beginning of this game, is my guess. The robot is able to correctly read the word and assess that I have won the game. It proceeds to draw the letters in the correct places. All letters are bubble letters, though as the robot draws, some letters show up better than others. The force admittance control adjusts at every point of the trajectory as it connects the dots. This means that the robot may not know it has fallen out of range until it reaches the next waypoint.

It is worth noting that the head looks "scruffy" due to the  active adjustment from the force feedback control. An added challenge that we chose as a group was to have the robot manipulate the marker without any additional aid from custom gripper attachments. While drawing, the robot will adjust to ensure that the admittance force is within the appropriate force range at each given waypoint on the trajectory. Due to flexion of the board, marker, and robot, this adjustment can be seen visually as it draws. 

### Future Work
Looking forward, there are some things that we seek to improve for optimized drawing and gameplay. First, a more highly tuned control loop that accounts for the robot having an easier time drawing upwards and sideways strokes than it does downward strokes would significantly improve the fidelity of the writing. Second, a more even distribution of some of the discrete points even on perfectly straight strokes, would allow for better evenness in the letters. 

For future work, the team envisions a mode where the robot changes markers in between each play depending on whether the play was correct or incorrect. Some of the code for this was already written though it could not be effectively implemented in the time given before presentation.

## Source code
For more information about how to run the game, our approach, and our code, please visit my Github and explore the code.

- [Github repo](https://github.com/Schelbert197/final-project-me495/tree/main)

