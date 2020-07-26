# DynamiCal

[![Build Status](https://travis-ci.com/tedklin/dynamical.svg?token=EQ1yVHxTi52hGw7TPsW5&branch=master)](https://travis-ci.com/tedklin/dynamical)

*DynamiCal* is a (WIP) header-only framework for the control and estimation of dynamical systems. 

It aims to be both educational and applicable, with a focus on learning about control system design / analysis through simulation, while simultaneously setting up a framework for potential real-world use.


## Motivation

This project was born out of a desire to further explore some ideas taught in an introductory class on systems ([EECS16B @Berkeley](https://inst.eecs.berkeley.edu/~ee16b/sp20/)). Concepts like state-space representation, stability, and feedback piqued my interest in particular. While 16B covered these from a mostly theoretical perspective, I wanted to see what the theory would translate to in software for real applications.

Although initial software design decisions were made with people coming from a 16B background in mind, the project has evolved beyond the scope of 16B. I started using the Astrom and Murray *Feedback Systems* text and other resources to self-study, and I plan to continually add concepts from relevant classes I take in the future.


## Current status

As of now, all functionality is for time-invariant systems.

Finished / mostly finished:
- Controllability
- Stability and Feedback
- Discretization
- Plant simulation with Gaussian noise generation

Under development / implemented but untested:
- Linearization
- Minimum energy trajectory generation
- Observer and linear Kalman filter in discrete-time
- Complete discrete-time State Feedback Controller simulation

Later down the road:
- ROS integration
- Linear Kalman filter design
- Optimization-based control?
- Nonlinear state estimation?

Specific examples and formal documentation have not been created yet, but the [tests](https://github.com/tedklin/dynamical/tree/master/tests) might give a basic idea of general usage.

An in-depth design document can be found [here](https://github.com/tedklin/dynamical/blob/master/docs/design.md).


## Dependencies

- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (Linear algebra)
- [autodiff]() (Automatic differentiation)

## References

- [EECS16B Spring 2020](https://inst.eecs.berkeley.edu/~ee16b/sp20/)
- [*Feedback Systems* by Astrom and Murray](http://www.cds.caltech.edu/~murray/amwiki/index.php?title=Main_Page)
- [*Kalman and Bayesian Filters in Python* by Roger Labbe](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python)
