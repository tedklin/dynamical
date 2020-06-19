# Design document

This document is intended to lay out the nontrivial software design decisions and nonobvious code in this project. It might be overkill for documentation purposes, but because this is my first larger-scale C++ project, I felt it was best to expose every decision made. It also helps me maintain my own sanity as the project grows.


## state_space

### plant.h
- Plant abstract template 
    - access control
        - public using declarations
            - Users of the class can avoid redundancy in specifying the various dimensions of the system.
            - Eliminate the verbosity of declaring Eigen library fixed matrix types.
        - public constant A, B, C, D
            - Eliminate the need for trivial getters ([C.131](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c131-avoid-trivial-getters-and-setters)).
        - private state and output
    - constructor
        - deleted default constructor 
            - In general, it doesn't make sense to create a Plant without any dynamics. 
            - Note that if users do need to create a Plant without any dynamics, they can still do so explicitly.
        - only one defined constructor, with default arguments for the C and D matrices
            - I don't have much experience with the actual implementation of state-space controllers in practice, so I don't really know if this was the best decision. The considerations I took for the design of the constructor(s) for this class are as follows:
                - In my school's (very) introductory systems class ([EECS16B at UC Berkeley](https://inst.eecs.berkeley.edu/~ee16b/sp20/)), the C and D matrices are ignored. It is assumed that all states are outputs (x_vec = y_vec, so C = Identity) and that there is no feedthrough (D = Zero).
                - On the [Wikipedia page for state-space controllers](https://en.wikipedia.org/wiki/State-space_representation), it is also noted that C and D are fairly commonly defaulted to the identity and zero matrices in practice. However, from my own personal observations (videos, papers, other code) it seems as if an explicit C is often pretty necessary (we often can't measure states directly due to feasibility / cost constraints). On the other hand, I haven't seen as strong of a need for the D matrix, so I guess D really can be safely ignored in many cases.
                - Having default arguments for C and D matrices adapts to both of the above situations.
                - It is more common to need to specify a nonzero initial state vector x than to define C and D explicitly. I also wanted to keep the ABCD matrices together for more intuitive instantiation (as opposed to sandwiching like ABxCD). As a result, the initial state vector goes as the first argument.
                    - It should be noted that this opens opportunity for error when initializing a Plant implementation (i.e. trying to initialize with arguments ABCD, forgetting that the A will become x, B will become A, and so forth).
- DiscretePlant template derived from Plant
    - Inheritance can be a little bit tricky with templates, see quick explanation [here](https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members).

