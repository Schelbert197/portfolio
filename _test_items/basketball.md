---
title: "Computer Vision Basketball Trainer"
author_profile: true
key: 15
excerpt: "Computer Vision, Motion Tracking, Google MediaPipe"
header:
  teaser: /assets/images/cv_project/Nash_ball_tracking.gif
classes: wide
gallery8879:
  - url: /assets/images/cv_project/output_pdf.png
    image_path: /assets/images/cv_project/output_pdf.png
    alt: "placeholder image 1"
    title: "Henry Make"
  - url: /assets/images/cv_project/srikanth_make.png
    image_path: /assets/images/cv_project/srikanth_make.png
    alt: "placeholder image 2"
    title: "Srikanth Make"
---

## Project Goal
This project aimed at using computer vision techniques to provide feedback on a the free throw shots executed by a basketball player. By tracking the motion of the ball and motion of the player's hands, the program outputs a PDF document offering data to help the player improve including the following:
- An overall score out of 100
- The release angle
- Graphs comparing the player's shot to one from Steve Nash

## Final Output

### Feedback
Below are examples of the final output for this project. 
{% include gallery id="gallery8879" %}

### Using the tracker
Using the 

![Rainbow]({{ site.url }}{{ site.baseurl }}/assets/images/cv_project/cv_input.png)

## Tracking the Player
To track the motion of the player, we used Google's Mediapipe program to get the trajectory of the key-points on the player's body. For our analysis, we focused on the motion of the hands/wrists as the player shot the ball.

## Tracking the Ball
Tracking the motion of the ball, unlike that of the player, proved to be a bit more complicated due to a few key challenges that often appear in computer vision applications

(list challenges)

## Comparing the Player's Motion
While using Mediapipe provides the trajectory of Steve Nash's and the player's motion, finding a way to meaningfully compare the trajectories and represent that as a score out of 100 required some creativity.