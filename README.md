# Extended Kalman Filter Projec
Self-Driving Car Engineer Nanodegree Program

In this project, a kalman filter is utilized to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 


## Requirements

* The Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).  
* Set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems


## Build & Run

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

## Components

* main.cpp - communicates with the Term 2 Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* FusionEKF.cpp - initializes the filter, calls the predict function, calls the update function
* kalman_filter.cpp- defines the predict function, the update function for lidar, and the update function for radar
* tools.cpp- function to calculate RMSE, the Jacobian matrix, and radar measurement estimation