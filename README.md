# DynamiCal

[![Build Status](https://travis-ci.com/tedklin/dynamical.svg?token=EQ1yVHxTi52hGw7TPsW5&branch=master)](https://travis-ci.com/tedklin/dynamical)

*DynamiCal* is a (WIP) control systems framework that aims to be both educational and applicable, with an intuitive interface and careful software design in C++.

*DynamiCal* was born out of a desire to further explore some ideas taught in an introductory class on systems ([EECS16B @Berkeley](https://inst.eecs.berkeley.edu/~ee16b/sp20/)). These concepts included things like state-space representation, stability, feedback, and more. While 16B covered these from a more theoretical perspective, I wanted to see how the theory translated to real life.

Of course, I don't actually have tons of physical mobile robots at my disposal (anymore), so this project focuses on learning / testing through simulations, while simultaneously setting up a framework for potential real-world use.

Although initial software design decisions were made while keeping in mind people coming from an only-16B background, the project has inevitably evolved beyond the scope of 16B. I started using the Astrom and Murray *Feedback Systems* text and other resources to self-study, and I plan to continually add concepts from relevant classes I take in the future.

Specific examples and formal documentation have not been created yet, but the [tests](https://github.com/tedklin/dynamical/tree/master/tests) might give a basic idea of general usage. An informal design doc can be found [here](https://github.com/tedklin/dynamical/blob/master/docs/design.md).


## Dependencies
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## References

- [EECS16B Spring 2020](https://inst.eecs.berkeley.edu/~ee16b/sp20/)
- [*Feedback Systems* by Astrom and Murray](http://www.cds.caltech.edu/~murray/amwiki/index.php?title=Main_Page)
- [ETH Zurich's *control-toolbox* library](https://github.com/ethz-adrl/control-toolbox)
