---
title: "Binary Sensor Bayes Update"
author_profile: true
key: 99
excerpt: "Bayes Update, Numpy, Matplotlib"
header:
  teaser: /assets/images/Die_cup_edited.gif
classes: wide
---


This project demonstrates the Baye's update algorithm to adjust belief of the location of a source over a finite space. 

## The Setup
To understand the algorithm, we assume that there is a robot with a binary sensor that is searching for the source of a signal. The sensor can only inform the robot that it does or does not sense the signal, but it cannot provide information on the strength of the signal. The source produces a signal that varies with distance from its location, but rather than diminishing as radial distance decreases, it follows the function f(x) = e^(-100 * (||x - source|| * 0.2)^2)

![Random_sample_plot]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p1.png)

The plot above shows a few key things:
- **White** : Shows the strength of the signal which can be interpreted as a probability of the sensor producing a positive read from 0 to 1
- **Green** : Shows the location of a randomly sampled robot placement where the sensor had a positive read
- **Red** : Shows the location of a randomly sampled robot placement where the sensor had a negative read
- **Blue** : Shows the ground truth location of the source of the signal

The sensor returns positive if the strength of the signal is higher than a random sample from a uniform distribution from 0 to 1. As shown the sensor mostly reads negative in the dark space and positive in the light space, though we see that this is not perfect which is similar to real life sensors. 

## Centerpoint Belief Function
Now that we know the sensor and the signal have been correctly modeled, it is time to calculate the belief of the location of the sensor. 

![Random_sample_plot2]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p2.png)

This plot demonstrats that the belief of the source of the signal can be visualized using a heatmap to show the highest probability location of the source. In this second image, we are seeing what the robot sees which is a set of random samples around the space and the robot's understanding of where the source most likely exists (modeled in white in this second image). The green dots do a good job to help visualize the source's signal from the first plot. While the robot does not know the ground truth, this guess is quite close.

EQUAITON
The above equation models the belief of the source. 

## Sparse Data "100 Shot" sensor
While it is easy to get a good sense of the location from lots of samples spread uniformly around the space, it is important to study how one sample affects the belief of the location of the source. 


FIX
![sparse_sample_plot]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p3.png)

In the above example, we assume each plot to show the belief of the location of the source with the orange dot acting as the FIRST and ONLY sample the robot has (meaning there is no prior information). The source, as shown in the first plot above, is still at (0.3, 0.4) but the robot does not know this. The middle plot here shows that the robot believes the source to be somewhere far away from the sample, which in this case is negative, but given no other information, it has no ability to narrow down the belief further. The left and rightmost plots are positive reads, and the robot able to narrow down the belief to two rings which results from the threshold of the source's signal which is where we learn the most. 

If this concept is confusing, think about the idea of walking down stairs in complete darkness. The most important information regarding where to place your foot comes from where the edge of the step is. We see that the model is able to capture the source quite well within the guess of possible locations (or heatmap), but again with this being a "single shot" read, we cannot narrow down without more information. 

## Successive Samples from One Location
In this next example it is worth exploring the question of "can the robot gain information from sampling somewhere when it cannot move?". The answer is in fact YES!

![successive_sample_plot]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p4.png)

The plots below show 10 sequential readings all taken from the same source. In this instance, the green heatmap shows the belief of where the sensor is, and the white shows the sensor's signal strength (as we saw in the first plot above). I have plotted it this way simply to make this easy to visualize. 

![frames]({{ site.url }}{{ site.baseurl }}/assets/images/Jackbox.png)

## Euler Lagrange Equations
![EL]({{ site.url }}{{ site.baseurl }}/assets/images/EL_eqn.png)

The Euler-Lagrange equations (shown in terms of L(q,qdot) above) are critical for finding the instantaneous accelerations of the bodies. By finding the body velocities of the box and the die, defining the inertial matrices, and calculating the kinetic energy and potential energy (so that we have the Lagrangian, L), the equation shown above can be calculated for each component of the configuration q. From these equations, the accelerations of the configuration are found, and the system is simulated using RK4 integration over a 0.01 second timestep.

## Constraints
The system is constrained by the impacts that keep the red die inside the blue box. Whenever an impact is detected by one of the 4 corners of the contacting one of the 4 edges of the box, the impact update equations will trigger in the simulation loop to reflect the appropriate dynamics of the system. Each collision is considered a fully elastic collision between bodies which can be seen from the video above.

## External Forces
There are two external forces acting on the system in this simulation. First, is a force that is equal to the force of the box due to gravity to keep it up. This force does not account for the impact of the die on the box, so it does fall slightly due to intertia as they impact each other. The second force is a rotation force on the box to give it an initial rotational velocity. This force has an inverse relationship with time so it provides an initial acceleration that decreases throughout the simulation.