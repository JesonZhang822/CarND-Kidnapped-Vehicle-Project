# Kidnapped Vehicle Project
Self-Driving Car Engineer Nanodegree Program

![ParticleFilter](./writeup/ParticleFilter.jpg)

## Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.

## Project Introduction
Our robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we will implement a 2 dimensional particle filter in C++. Our particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step our filter will also get observation and control data.

## Write Up



## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh


Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

* sense noisy position data from the simulator :` ["sense_x"],["sense_y"],["sense_theta"]`

* get the previous velocity and yaw rate to predict the particle's transitioned state :`["previous_velocity"] , ["previous_yawrate"]`

* receive noisy observation data from the simulator, in a respective list of x/y values :`["sense_observations_x"] ["sense_observations_y"]`

OUTPUT: values provided by the c++ program to the simulator

* best particle values used for calculating the error evaluation : `["best_particle_x"],["best_particle_y"],["best_particle_theta"]`

* Optional message data used for debugging particle's sensing and associations

* for respective (x,y) sensed positions ID label :`["best_particle_associations"]`

* for respective (x,y) sensed positions :

`["best_particle_sense_x"] <= list of sensed x positions`

`["best_particle_sense_y"] <= list of sensed y positions `
