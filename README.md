# DynamiCal

[![Build Status](https://travis-ci.com/tedklin/dynamical.svg?token=EQ1yVHxTi52hGw7TPsW5&branch=master)](https://travis-ci.com/tedklin/dynamical)

*DynamiCal* is a (WIP) control systems framework that aims to be both educational and applicable, with an intuitive interface and careful software design in C++.

This project was born out of a desire to further explore some ideas taught in an introductory class on systems ([EECS16B @Berkeley](https://inst.eecs.berkeley.edu/~ee16b/sp20/)). Concepts like state-space representation, stability, and feedback piqued my interest in particular. While 16B covered these from a mostly theoretical perspective, I wanted to see what the theory would translate to in software for real applications.

Of course, I don't actually have tons of physical mobile robots at my disposal (anymore), so this project focuses on learning / testing through simulations, while simultaneously setting up a framework for potential real-world use.

Although initial software design decisions were made with people coming from a 16B background in mind, the project has evolved beyond the scope of 16B. I started using the Astrom and Murray *Feedback Systems* text and other resources to self-study, and I plan to continually add concepts from relevant classes I take in the future.

Specific examples and formal documentation have not been created yet, but the [tests](https://github.com/tedklin/dynamical/tree/master/tests) might give a basic idea of general usage. An informal design doc can be found [here](https://github.com/tedklin/dynamical/blob/master/docs/design.md).


## Dependencies
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## References

- [EECS16B Spring 2020](https://inst.eecs.berkeley.edu/~ee16b/sp20/)
- [*Feedback Systems* by Astrom and Murray](http://www.cds.caltech.edu/~murray/amwiki/index.php?title=Main_Page)
