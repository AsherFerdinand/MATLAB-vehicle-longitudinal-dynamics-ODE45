# MATLAB-vehicle-longitudinal-dynamics
# Vehicle Longitudinal Dynamics

A MATLAB-based simulation of vehicle longitudinal dynamics, focusing on vehicle acceleration, tire forces, wheel slip, load transfer, and traction limits.

## Overview

This project models a vehicle driving on a road with a given gradient. The vehicle dynamics are solved numerically to study how the vehicle responds to driving torque and tire-road interaction.

The simulation includes:

* Vehicle and wheel masses
* Wheel rotational inertia
* Front and rear axle loads
* Longitudinal tire forces
* Wheel speeds and vehicle speed
* Tire longitudinal deformation
* Wheel slip
* Vehicle acceleration
* Driving and braking torque distribution
* Tire-road friction utilization
* Maximum achievable equivalent road gradient

## Simulation

The vehicle model is solved using MATLAB's `ode45` numerical differential-equation solver.

The main simulation states are:

1. Vehicle longitudinal velocity
2. Front wheel angular velocity
3. Rear wheel angular velocity
4. Front tire longitudinal deformation
5. Rear tire longitudinal deformation
6. Vehicle position

The simulation results are visualized through plots showing vehicle speed, wheel speed, tire forces, axle loads, slip, acceleration, tire deformation, and traction limits.


## Requirements

* MATLAB
* MATLAB ODE45 solver
* The required vehicle and tire model functions included in this repository

## Goal

The goal of this project is to build a vehicle longitudinal dynamics model from scratch and develop an understanding of the relationship between vehicle motion, tire forces, wheel slip, load transfer, and available traction.

## Status

🚧 Work in progress
