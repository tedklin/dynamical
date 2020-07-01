# Design document

This living document is intended to lay out some design decisions and non-obvious code. It's for keeping track of running TODOs related to design, as well as the opportunity to fix bad C++ practices / wrong assumptions about control system design. It doesn't cover all functionality.


## Some driving design goals

1. User-safety
    - Minimize undefined behavior.
    - Force users to state what they want as explicitly as possible.
    - Every class and every function should do exactly what their names say.
    - Maintain const-correctness.
    - Tests! Lots of tests!
2. Modularity
    - Break down systems into their smallest components while still maintaining realistic abstraction for usability.
    - Levels of functionality based on external dependencies and other overhead.
        - Each level of functionality should work on its own and not be too entangled with higher levels of functionality, especially ones that introduce external dependencies.
        - Prefer to define non-core functionality as generic functions that act on a given object instead of member functions contained within each core object.
3. Efficiency
    - Optimizations like enabling move operations where it makes sense.
        - Keep in mind optimizations the compiler itself can make too, like copy elision / RVO.
    - Learn how to use tools like sanitizers and profilers. 


## Levels of functionality

1. Discrete-time linear control system design
2. Linear systems analysis
    - with limited functionality for continuous-time systems.
3. Nonlinear control system design (onboard linearization)
4. Integrated solvers and implementations for optimal control (LQR, MPC, etc.)


## High-level TODOs
- TEMPLATES ARE EVERYWHERE
    - TODO: how to make this cleaner???
- The current implementation of most of the functions returns by value. This could be an expensive copy operation if the data structure is large.
    - An alternative is returning by reference, but undefined behavior would result if object to be returned is defined and created locally (in the function itself). Returning by reference would require the user to do something like manually define their object first, then pass it by reference as an argument to the function.
    - Another alternative is returning a smart pointer (like a unique_ptr factory), but this would also require the user to do more work (and have an understanding of smart pointers).
    - TODO: read up on return value optimization to see if this is really a problem at all.
        - i think i understand copy elision eliminating the copy action into the temporary, but it seems like there's still a copy-assign happening at the call site. when a function returns a local object by value, is there a way to just directly use move semantics? or does the compiler do that implicitly?
    - TODO: explore more alternate possibilities to avoid expensive copy operation while maintaining ease of use.
- Some of the decisions I'm making are taking into heavy consideration possible usage with my school's (very) introductory systems class ([EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/)). This might result in some features that don't make sense for real world systems.
    - TODO: decide if satisfying 16B constraints is really worth it.


## High-level design notes
- Most of the math is implemented with techniques we learned in EECS16B, but the overall control system structure is based on [parts of Astrom and Murray (A&M)](http://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-outputfbk_28Sep12.pdf). There are some (mostly trivial) clashes in convention between the two sources:
    - The K matrix for feedback has a positive sign in this project, which is the same as [16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/13a.pdf). However, it has a negative sign in [A&M](http://www.cds.caltech.edu/~murray/amwiki/index.php?title=State_Feedback).
    - A&M notes that it prefers "reachability" to "controllability", and they're technically not the same thing, but in the context of 16B they're the same thing and we use the term "controllability".
- Does the order in which *x* and *y* are updated matter (see Plant and Observer)? My current implementation updates y first.
    - It seems like y should be updated before x based on the Wikipedia pages for [*state-space*](https://en.wikipedia.org/wiki/State-space_representation) and [*state observer*](https://en.wikipedia.org/wiki/State_observer), but most implementations I've seen update x first ("predict then correct" Kalman filter concept). 
    - It might not matter that much when running at high clock rates as long as it's consistent as to which comes first, but making a note regardless.


## File-by-file implementation details

### state_space/plant.hpp
*Plant*
- *namespace dynamical::lti*
- *type: abstract class template*
- Design with inheritance was used despite there only being two children of Plant. This is because there are several instances where the distinction between discrete-time and continuous-time plants are ignored (e.g. controllability and observability), but there are also significant differences between the two (discrete-time systems are characterized by difference equations whereas continuous-time plants are characterized by differential equations).
- access control
    - public using declarations for type names
        - Users can avoid errors in specifying the various dimensions of the system matrices.
        - Users can easily create special matrices (through functions defined by Eigen) of the correct type.
        - Eliminate the verbosity of declaring Eigen library fixed matrix types.
    - public constant A, B, C, D
        - Eliminate the need for trivial getters ([C.131](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c131-avoid-trivial-getters-and-setters)). It doesn't make sense for the system dynamics of a plant to be modified after initialization (LTI).
    - protected state and output
        - Standard encapsulation.
- constructor
    - deleted default constructor
        - In general, it doesn't make sense to create a Plant without any dynamics.
        - Note that if users do need to create a Plant without any dynamics, they can still do so explicitly.
    - one defined constructor, with the initial state vector as the first argument and default arguments for the C and D matrices.
        - In EECS16B, the concept of output is not taught and the C and D matrices are ignored. It is assumed that all states are outputs (x_vec = y_vec, so C = Identity) and that there is no feedthrough (D = Zero).
            - This is also why the template parameter num_outputs has a default argument of num_states.
        - It is more common to need to specify a nonzero initial state vector x than to define D explicitly. I also wanted to keep the ABCD matrices together for more intuitive instantiation (as opposed to sandwiching like A,B,x,C,D). As a result, the initial state vector goes as the first argument.
            - It should be noted that this opens up opportunity for error when initializing a Plant type implementation (i.e. trying to initialize a DiscretePlant with arguments ABCD, forgetting that the A will become x, B will become A, and so forth). I'm fairly certain the compiler would throw an error if this does happen, but I haven't tested this.
- copy-control.
    - none explicitly defined (Rule of Zero).
    - all are synthesized(?)

*DiscretePlant*
- *namespace dynamical::lti*
- *type: class template implementation of Plant*
- The first using declaration lets us inherit all non copy-control constructors directly from Plant.
- Inheritance can be a little bit tricky with templates, see quick explanation [here](https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members) for why we need an explicit *this->* and typename declarations.

*ContinuousPlant*
- *namespace dynamical::lti*
- *type: class template implementation of Plant*
- TODO: check if synthesizing a lambda on every call to UpdateSim() significantly hurts performance or if it's better to just define a separate function representing the differential equations and point to it.
- TODO: implement Update() as real-time update?
    - std::chrono
        - https://en.cppreference.com/w/cpp/chrono/duration
        - https://en.cppreference.com/w/cpp/chrono/time_point


### state_space/controller.hpp

*Controller*
- *namespace dynamical::lti*
- *type: class template*

*Observer*
- *namespace dynamical::lti*
- *type: class template*
- An observer conceptually contains an internal model of the plant it's observing. The observer naturally shouldn't have the power to directly "update" the actual plant's current state *x*. Instead the observer keeps its own internal estimate of the state *x_hat* and updates that.
    - Using a pointer to the observed plant would probably be more efficient on initialization, but hides the intention / role of the observer. I opted to just copy the A, B, C, and D matrices (encoding the dynamics of the system) from the given plant to observe.
- The observer introduces a new term (*L(y - Cx_hat)*) to the original propagation equations Ax + Bu. We need to differentiate between observers for discrete-time systems and continuous-time systems because the propogation equations are different types (discrete difference equation vs continuous differential equation).
    - I couldn't think of any instances where the distinction between discrete-time and continuous-time observers can be ignored, so inheritance didn't seem appropriate. An enum seems like the easiest way to keep track of what type of system the observer is dealing with.
    - However, at the moment, I'm not sure about the practicality of actually using a complete state feedback controller based in continuous-time (numerical integration / time sync errors add up), so the current implementation is just for discrete time.


### state_space/analysis.hpp

Overall notes:
- The current implementation looks pretty cluttered. But template definitions shouldn't be separated from their declarations, so I'm not really sure how to make this better.
    - TODO: is there a better way to organize this or just leave it as is?
- The Eigen library [doesn't play well with *auto* type deduction](https://eigen.tuxfamily.org/dox/TopicPitfalls.html), so I spelt out matrix types even where using *auto* would make sense. This should result in better-guaranteed correctness at the expense of verbosity (and maybe readability).

*get_controllability_matrix*
- *namespace dynamical::lti::analysis*
- *type: function template*
- I'm not entirely sure what goes on behind the scenes when I use a template argument (*state_dim*) directly as a default argument for the *num_steps* parameter, but tests have shown that this works.
    - TODO: understand this better.

*get_observability_matrix*
- *namespace dynamical::lti::analysis*
- *type: function template*

*is_stable*
- *namespace dynamical::lti::analysis*
- *type: function template*

*discretize*
- *namespace dynamical::lti::analysis*
- *type: function template*
- This function implements continuous-time plant discretization as taught in [EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/). 
    - This method uses the eigenbasis (which isn't guaranteed to be real even if your matrix is real) for diagonalization. At first I tried to optimize for cases where the eigenbasis actually is real to avoid the overhead of *std::complex*, but it became a long struggle with the Eigen library's type deduction/conversion rules. I ended up just sticking with *std::complex< double >* for everything, so all DiscretePlant instances created by this function have a complex double Scalar type regardless of what ContinuousPlant is passed in.
- TODO: overload with an implementation that doesn't rely on an invertible eigenbasis?
