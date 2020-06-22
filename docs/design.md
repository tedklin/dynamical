# Design document

This document is intended to lay out nontrivial software design decisions and non-obvious code. Since *dynamical* is my first larger-scale C++ project, I decided it's best to expose the reasoning behind somewhat-significant design decisions made, for self-reflection purposes and the opportunity to fix bad C++ practices / wrong assumptions about the nature of control system design.

**This (admittedly pedantic) document is not geared towards user documentation purposes and shouldn't be used as such.**

## Some driving design goals

1. Safety
    - Minimize undefined behavior.
        - Force users to state what they want as explicitly as possible.
    - Every class and every function should do exactly what their names say.
    - Maintain const-correctness.
    - Tests! Lots of tests!
    - ASan?
2. Modularity and efficiency
    - Break down systems into their smallest components while still maintaining realistic abstraction for usability.
    - Prefer to define generic functions that act on a given object instead of member functions contained within each object.
3. User-focused
    - Keep in mind the applications a library like this could be used for, and design around that.

## state_space

### plant.h
*Plant*
- *namespace dynamical*
- *type: template abstract class*
- access control
    - public using declarations for type names
        - Users can avoid errors in specifying the various dimensions of the system matrices.
        - Users can easily create special matrices (through functions defined by Eigen) of the correct type.
        - Eliminate the verbosity of declaring Eigen library fixed matrix types.
    - public constant A, B, C, D
        - Eliminate the need for trivial getters ([C.131](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c131-avoid-trivial-getters-and-setters)).
    - protected state and output
        - Encapsulation for safety.
- constructor
    - deleted default constructor
        - In general, it doesn't make sense to create a Plant without any dynamics.
        - Note that if users do need to create a Plant without any dynamics, they can still do so explicitly.
    - only one defined constructor, with default arguments for the C and D matrices
        - I don't have much experience with implementation of state-space controllers in practice, so I don't really know if this was the best decision. The considerations I took for the design of the constructor(s) for this class are as follows:
            - In my school's (very) introductory systems class ([EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/)), the C and D matrices are ignored. It is assumed that all states are outputs (x_vec = y_vec, so C = Identity) and that there is no feedthrough (D = Zero).
            - The [Wikipedia page for state-space controllers](https://en.wikipedia.org/wiki/State-space_representation) also notes that C and D are fairly commonly defaulted to the identity and zero matrices in practice. However, from my own personal observations (videos, papers, other code) it seems as if an explicit C is often pretty necessary (we often can't measure states directly due to feasibility / cost constraints). On the other hand, I haven't seen as strong of a need for the D matrix, so D might be safe to ignore in many cases.
            - Having default arguments for C and D matrices adapts to all of the above situations.
            - It is more common to need to specify a nonzero initial state vector x than to define C and D explicitly. I also wanted to keep the ABCD matrices together for more intuitive instantiation (as opposed to sandwiching like A,B,x,C,D). As a result, the initial state vector goes as the first argument.
                - It should be noted that this opens up opportunity for error when initializing a Plant type implementation (i.e. trying to initialize a DiscretePlant with arguments ABCD, forgetting that the A will become x, B will become A, and so forth). I'm fairly certain the compiler would throw an error if this does happen, but I haven't tested this.

*DiscretePlant*
- *namespace dynamical*
- *type: template class implementation of Plant*
- The first using declaration lets us inherit all non copy-control constructors directly from Plant.
- Inheritance can be a little bit tricky with templates, see quick explanation [here](https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members) for why we need an explicit *this->* and typename declarations.

*ContinuousPlant*
- *namespace dynamical*
- *type: template class implementation of Plant*

### analysis.h

overall notes
    - The Eigen library doesn't play well with *auto* type deduction, so I spelt out matrix types even where using *auto* would make sense. A small tradeoff between verbosity and guaranteed correctness.

*get_controllability_matrix*
- *namespace dynamical::analysis*
- *type: template function*
- I'm not entirely sure what goes on behind the scenes when I use a template argument (*state_dim*) directly as a default argument for the *num_steps* parameter, but tests have shown that this works.
    - TODO: understand this better.
- The current implementation returns the controllability matrix by value. This could be an expensive operation if the matrix is large. An alternative is returning by reference, but undefined behavior would result because the controllability matrix is defined and created locally (in the function itself). Returning by reference would require the user to do something like manually define their own controllability matrix first, then pass it by reference as an argument to the function. Since this is a generic analysis function that would probably not be called in real-time implementations (it will most likely be run offline first), the inconvenience of forcing the user to declare their own type / dimensions for the controllability matrix outweighs the cost of the copy operation.
    - TODO: explore alternate possibilities to avoid expensive copy operation while maintaining ease of use.

*is_controllable*
- *namespace dynamical::analysis*
- *type: template function*

*discretize*
- *namespace dynamical::analysis*
- *type: template function*
- This function implements continuous-time plant discretization as taught in [EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/). 
    - This method uses the eigenbasis (which isn't guaranteed to be real even if your matrix is real) for diagonalization. At first I tried to optimize for cases where the eigenbasis actually is real to avoid the overhead of *std::complex*, but it became a long struggle with the Eigen library's type deduction/conversion rules. I ended up just sticking with *std::complex< double >* for everything, so all DiscretePlant instances created by this function have a complex double Scalar type regardless of what ContinuousPlant is passed in.
