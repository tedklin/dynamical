# Design document

This living document is intended to lay out software design decisions and non-obvious code. Since *dynamical* is my first larger-scale C++ project, I decided it's best to expose the reasoning behind even trivial design decisions made. This is for keeping track of running TODOs related to design, as well as the opportunity to fix bad C++ practices / wrong assumptions about control system design. It admittedly got pretty pedantic :0


## Some driving design goals

1. User-safety
    - Minimize undefined behavior.
    - Force users to state what they want as explicitly as possible.
    - Every class and every function should do exactly what their names say.
    - Maintain const-correctness.
    - Tests! Lots of tests!
2. Modularity
    - Break down systems into their smallest components while still maintaining realistic abstraction for usability.
        - e.g. each block type in a typical control system diagram should be its own class.
    - Levels of functionality based on external dependencies and other overhead.
        - Each level of functionality should work on its own and not be too entangled with higher levels of functionality, especially ones that introduce external dependencies.
        - Prefer to define non-core functionality as generic functions that act on a given object instead of member functions contained within each object.
3. Efficiency
    - Optimizations like enabling move operations where it makes sense.
        - Keep in mind optimizations the compiler itself can make too, like copy elision / RVO.
    - Learn how to use tools like sanitizers and profilers. 
4. Application-focused
    - Keep in mind the applications a library like this could be used for, and design around that.


## Levels of functionality

1. Real-time discrete and "continuous" (with onboard discretization) linear control systems
2. Linear systems analysis
4. Real-time nonlinear control systems (onboard linearization)
4. Built-in calculations for optimal control (LQR, etc.)


## High-level TODOs
- TEMPLATES. ARE. EVERYWHERE.
    - TODO: how to make this cleaner???????
- The current implementation of most of the functions returns by value. This could be an expensive copy operation if the data structure is large.
    - An alternative is returning by reference, but undefined behavior would result if object to be returned is defined and created locally (in the function itself). Returning by reference would require the user to do something like manually define their object first, then pass it by reference as an argument to the function.
    - Another alternative is returning a smart pointer (like a unique_ptr factory), but this would also require the user to do more work (and have an understanding of smart pointers).
    - TODO: read up on return value optimization to see if this is really a problem at all.
        - i understand copy elision, but when a function returns a local object by value, does it do so by copying or does it implictly use move?
        - **this question extends to a lot of my currently implemented code.**
    - TODO: explore more alternate possibilities to avoid expensive copy operation while maintaining ease of use.
- Some of the decisions I'm making are taking into heavy consideration possible usage with my school's (very) introductory systems class ([EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/)). This might result in some features that don't make sense for real world systems.
    - TODO: decide if satisfying 16B constraints is really worth it.


## High-level design notes
- Most of the math is implemented with techniques we learned in 16B, but the overall control system structure is based on [parts of Astrom and Murray (A&M)](http://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-outputfbk_28Sep12.pdf). However, there are some clashes in convention between the two sources:
    - The K matrix for feedback has a positive sign in this project, which is the same as [16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/13a.pdf). However, it has a negative sign in [A&M](http://www.cds.caltech.edu/~murray/amwiki/index.php?title=State_Feedback).


## File-by-file implementation details

### state_space/plant.hpp
*Plant*
- *namespace dynamical*
- *type: template abstract class*
- access control
    - public using declarations for type names
        - Users can avoid errors in specifying the various dimensions of the system matrices.
        - Users can easily create special matrices (through functions defined by Eigen) of the correct type.
        - Eliminate the verbosity of declaring Eigen library fixed matrix types.
    - public constant A, B, C, D
        - Eliminate the need for trivial getters ([C.131](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c131-avoid-trivial-getters-and-setters)). It doesn't make sense for the system dynamics of a plant to be modified after initialization.
    - protected state and output
        - Standard encapsulation.
- constructor
    - deleted default constructor
        - In general, it doesn't make sense to create a Plant without any dynamics.
        - Note that if users do need to create a Plant without any dynamics, they can still do so explicitly.
    - one defined constructor, with the initial state vector as the first argument and default arguments for the C and D matrices.
        - I don't have much experience with implementation of state-space controllers in practice, so I don't really know if this was the best decision. The considerations I took for the design of the constructor(s) for this class are as follows:
            - In EECS16B, the concept of output is not taught and the C and D matrices are ignored. It is assumed that all states are outputs (x_vec = y_vec, so C = Identity) and that there is no feedthrough (D = Zero).
                - This is also why the template parameter num_outputs has a default argument of num_states.
            - The [Wikipedia page for state-space controllers](https://en.wikipedia.org/wiki/State-space_representation) also notes that C and D are fairly commonly defaulted to the identity and zero matrices in practice. However, from my own personal observations (videos, papers, other code) it seems as if an explicitly defined C is often pretty necessary (we often can't measure states directly due to feasibility / cost constraints). I haven't seen as strong of a need for an explicit D matrix (and it seems like it reduces some computations), so there may be many cases where only A, B, and C are defined.
            - It is more common to need to specify a nonzero initial state vector x than to define C and D explicitly. I also wanted to keep the ABCD matrices together for more intuitive instantiation (as opposed to sandwiching like A,B,x,C,D). As a result, the initial state vector goes as the first argument.
                - It should be noted that this opens up opportunity for error when initializing a Plant type implementation (i.e. trying to initialize a DiscretePlant with arguments ABCD, forgetting that the A will become x, B will become A, and so forth). I'm fairly certain the compiler would throw an error if this does happen, but I haven't tested this.
- copy-control.
    - none explicitly defined (Rule of Zero).
    - all are synthesized(?)

*DiscretePlant*
- *namespace dynamical*
- *type: template class implementation of Plant*
- The first using declaration lets us inherit all non copy-control constructors directly from Plant.
- Inheritance can be a little bit tricky with templates, see quick explanation [here](https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members) for why we need an explicit *this->* and typename declarations.

*ContinuousPlant*
- *namespace dynamical*
- *type: template class implementation of Plant*
- TODO: explore accuracy / efficiency of integration methods.
- TODO: implement Update() as real-time update?
    - std::chrono
        - https://stackoverflow.com/questions/728068/how-to-calculate-a-time-difference-in-c
        - https://en.cppreference.com/w/cpp/chrono/duration
        - https://en.cppreference.com/w/cpp/chrono/time_point


### state_space/controller.hpp


### state_space/analysis.hpp

overall notes
- The current implementation is pretty cluttered and not pleasing to the eyes. But template definitions shouldn't be separated from their declarations, so I'm not really sure how to make this better.
    - TODO: is there a better way to organize this or just leave it as is?
- The Eigen library [doesn't play well with *auto* type deduction](https://eigen.tuxfamily.org/dox/TopicPitfalls.html), so I spelt out matrix types even where using *auto* would make sense. This should result in better-guaranteed correctness at the expense of verbosity (and maybe readability).

*get_controllability_matrix*
- *namespace dynamical::analysis*
- *type: template function*
- I'm not entirely sure what goes on behind the scenes when I use a template argument (*state_dim*) directly as a default argument for the *num_steps* parameter, but tests have shown that this works.
    - TODO: understand this better.

*is_controllable*
- *namespace dynamical::analysis*
- *type: template function*

*get_observability_matrix*
- *namespace dynamical::analysis*
- *type: template function*

*is_observable*
- *namespace dynamical::analysis*
- *type: template function*

*stability_helper*
- *namespace dynamical::analysis*
- *type: template function*

*is_stable*
- *namespace dynamical::analysis*
- *type: template function*
- TODO: figure out if four overloads for this method is really necessary.
    - but don't cut down on overloads if it means losing guarantees on correctness.

*discretize*
- *namespace dynamical::analysis*
- *type: template function*
- This function implements continuous-time plant discretization as taught in [EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/). 
    - This method uses the eigenbasis (which isn't guaranteed to be real even if your matrix is real) for diagonalization. At first I tried to optimize for cases where the eigenbasis actually is real to avoid the overhead of *std::complex*, but it became a long struggle with the Eigen library's type deduction/conversion rules. I ended up just sticking with *std::complex< double >* for everything, so all DiscretePlant instances created by this function have a complex double Scalar type regardless of what ContinuousPlant is passed in.
