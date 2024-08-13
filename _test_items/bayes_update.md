---
title: "Binary Sensor Bayes' Update"
author_profile: true
key: 98
excerpt: "Bayes' Update, Numpy, Matplotlib"
header:
  teaser: /assets/images/Die_cup_edited.gif
classes: wide
---


This project demonstrates the Bayes' update algorithm to adjust belief of the location of a source over a finite space. 

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
While it is easy to get a good sense of the location from lots of samples spread uniformly around the space, it is important to study how sampling in one location affects the belief of the location of the source. It is worth exploring the question of "can the robot gain information from sampling somewhere when it cannot move?". The answer is in fact YES!

![sparse_sample_plot]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p3.png)

In the above example, we assume each plot to show the belief heatmap of the location of the source with the orange dot acting as only sample location the robot has. The dot is orange to denote that this could be a mix of positive or negative sensor readings and is not simply one measurement. The source, as shown with the blue "x" marked in the legend, is still at (0.3, 0.4) but the robot does not know this. The robot's goal is to get the blue "x" in the narrowest white portion possible. The middle plot here shows that the robot believes the source to be somewhere far away from the sample, but with the samples repeatedly appearing negative, the robot cannot narrow down the belief further. The left and rightmost plots are mixed reads which create thresholding allowing the robot to narrow down the belief to two rings. It is in this thresholding that the most information is gained. 

If this concept is confusing, think about the idea of walking down stairs in complete darkness. The most important information regarding where to place your foot comes from where the edge of the step is rather than the face of the step or no step at all. We see that the model is able to capture the source quite well within the rings, but again with this being constrained to one location, we cannot narrow down our belief without more information. 

## 10 Successive Samples from One Location
To better understand what is happening in the last visuals the below plot models 10 sequential samples all taken from the same location [0.17, 0.51] where the belief gets updated after each sample reading is taken. 

![successive_sample_plot]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p4.png)

In these plots, the green heatmap shows the belief of where the sensor is, and the white shows the sensor's signal strength (as we saw in the first plot above). I have plotted it this way simply to make this easy to visualize. 

In these examples, the beleif begins as a fuzzy green ring that narrows with each positive results, but suddenly a negative result in the 6th plot "splits" the belief into two thinner concentric rings that appear to create the inverse of the original one. This is because a sensor reading that flips is more informative. Another real-life example of this would be that if you want to know a person's location in a room, it is easy to guess where they are if you know they just crossed the threshold letting you know they're near the doorway (which is represented by the flipped sensor reading). 

While this plot is able to sharpen the rings considerably, there is still ambiguity due to the robot's inability to move, so the rings, as we saw in the last example, cannot be narrowed to a focused point. 

## 10 Successive Samples While Moving
Since the last example was restricted in movement, it is now worth exploring how the belief changes as the robot takes sensor measurements while moving. The below plots show 10 readings while the robot moves in the space, and like before, the green heatmap represents the belief or probability of the source being in that location.

![moving_robot]({{ site.url }}{{ site.baseurl }}/assets/images/active_learning/hw1_p5.png)

While the sampling only in one location yielded crisp rings, moving the sensor's sampling locations while also having the sensor results change allows the belief to be narrowed down to a few key areas which is also reflected in the scale of the green bar to right of each plot as the probability increases.

## Closing Thoughts
While this is very useful in the application of robotics for mapping and navigation, Bayes' update has applications beyond this from predicting geographical likelihood of crime to predicting which vaccines would be most effective.