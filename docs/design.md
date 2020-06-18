# Design document

This document is intended to lay out all software design decisions made for this project. Because this is my first larger-scale C++ project, I felt it was best to expose every decision made for my future self / any curious readers.

The layout will roughly be as follows:

- ***decision***
    - ***justifications***


## state_space

### Plant.h
- access control
    - public using declarations
        - Users of the class can avoid the redundancy of specifying the various dimensions of the system.
        - Eliminate the verbosity of declaring Eigen library fixed matrix types.
    - public constant A, B, C, D
    - private state and output
- constructor
    - deleted default constructor 
        - In general, it doesn't make sense to create a Plant without any dynamics. 
        - Note that if users do need to create a Plant without any dynamics, they can still do so explicitly.
    - only one defined constructor, with default arguments for the C and D matrices
        - I don't have much experience with the actual implementation of state-space controllers in practice, so I don't really know if this was the best decision. The considerations I took for the design of the constructor(s) for this class are as follows:
            - In my school's (very) introductory systems class ([EECS16B at UC Berkeley](https://inst.eecs.berkeley.edu/~ee16b/sp20/)), the C and D matrices are ignored. It is assumed that all states are outputs (x_vec = y_vec, so C = Identity) and that there is no feedthrough (D = Zero).
            - On the [Wikipedia page for state-space controllers](https://en.wikipedia.org/wiki/State-space_representation), it is also noted that C and D are fairly commonly ignored in practice. However, from my own personal observations (videos, reading other's code) it seems as if C is pretty necessary most of the time because of the need for an observer (we often can't measure all of our states directly due to feasibility / cost constraints). On the other hand, I haven't seen as strong of a need for the D matrix.
            - Default arguments for C and D matrices is adaptable to both of the above situations.

