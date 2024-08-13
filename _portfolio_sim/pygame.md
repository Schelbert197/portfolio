---
title: "Python Alien Game: Personal Coding Project"
author_profile: true
key: 45
toc: true
excerpt: "Python3, Pygame"
header:
  teaser: /assets/images/Python_Alien_Game.gif
classes: wide
---
## Gameplay Demo
<iframe width="560" height="315" src="https://www.youtube.com/embed/DpZxz6lg5Dc?si=-vOJwR3fyCvFsIe5" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Project Overview
The purpose of this project was to teach myself Python over the course of a summer. This project was a challenge that came as part of a book (cited below), and 

## Project goals
- Create a playable game using python.
- Learn traditional object oriented programming functionality.
- Adhere to common coding practices found in the workplace like pep8.

## Basic Structure
The project employs various classes to make the game work. The alien ships and projectiles are all sprites that allow the game to smoothly capture the collision between the objects. 

The game keeps track of the following items:
- Number of lives left (there are 3 ships in reserve)
- Alien point values (they increase with each level passed)
- Alien speed (increases throughout levels)
- Highest score per session

In the demo video, the gameplay shows the player winning the first level which increments the level, and also increases the point values of each alien. The Aliens also begin to move faster as the game progresses, though it is only a small difference in the first few levels. If the player decides to play another round, the high score will be tracked for next time. 

## Future work
If I want to improve this game, I would do two main things:
1. Write a file that tracks the high score even when a new session is played. 
2. Create different colored aliens worth different amounts.
3. Have the aliens begin to drop projectiles at random intervals after a level or two to make the game more challenging.

Overall, this was an enjoyable project and a great way to teach myself the foundation for object-oriented programming in Python!

## Book Reference
- [Python Crash Course by Eric Matthes (2nd ed.)](https://search.library.northwestern.edu/permalink/01NWU_INST/h04e76/alma9981878688802441)

## Source Code
- [Github Repository](https://github.com/Schelbert197/alien_game)