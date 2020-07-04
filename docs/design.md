# Design document

This living document is intended to lay out some design decisions and non-obvious code. It's for keeping track of running TODOs related to design, as well as the opportunity to fix bad C++ practices / wrong assumptions about control system design. It doesn't cover all functionality and isn't intended to be used as formal documentation.


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


## Levels of functionality (currently planned)

1. Discrete-time linear control system design
2. Linear systems simulation and analysis
3. Nonlinear control system design (onboard linearization)
4. Integrated solvers and implementations for optimal control (LQR, MPC, etc.)


## Overall control system design notes
- Most of the math is implemented with techniques we learned in [EECS16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/), but the overall control system structure is based on [parts of Astrom and Murray (A&M)](http://www.cds.caltech.edu/~murray/books/AM08/pdf/am08-outputfbk_28Sep12.pdf). There are some (mostly trivial) clashes in convention between the two sources:
    - The K matrix for feedback has a positive sign in this project, which is the same as [16B](https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/13a.pdf). However, it has a negative sign in [A&M](http://www.cds.caltech.edu/~murray/amwiki/index.php?title=State_Feedback).
    - A&M notes that it prefers "reachability" to "controllability", and they're technically not the same thing, but in the context of 16B they're the same thing and we use the term "controllability".
- I'm not really sure if the order I'm executing things is proper. The current implementation is mostly based on the equations in the Wikipedia pages for [*state-space*](https://en.wikipedia.org/wiki/State-space_representation) and [*state observer*](https://en.wikipedia.org/wiki/State_observer).


## Overall C++ design notes
- Heavy use of templates.
    - Things get pretty messy at times (see lti/analysis.hpp) but I don't really see another rational way to express the dependency on system dimensions (number of states, inputs, and outputs).
- Namespaces designed to be very specific (around three levels) to signify intention clearly.
    - I'm not sure if this is generally considered good or bad practice, but I feel like it's made code easier to follow.
- Avoided *auto* with Eigen types, even where it makes sense / passes tests.
    - Reasoning
        - The Eigen library [doesn't play well with *auto* type deduction](https://eigen.tuxfamily.org/dox/TopicPitfalls.html)
        - Better-guaranteed correctness if everything is just spelt out.
    - Cons:
        - Verbosity (and most likely readability).
- Public using declarations for type names of matrices (e.g. A_MatrixType, x_VectorType) were included inside most of my self-defined classes that deal with Eigen matrices.
    - Reasoning:
        - Eliminate the verbosity of declaring Eigen fixed matrix types everywhere.
        - Intuitively matches to template parameters of corresponding class.
        - Users can avoid errors in specifying the various dimensions of the system matrices.
        - Users can easily create ["special" Eigen matrices](https://eigen.tuxfamily.org/dox/group__TutorialAdvancedInitialization.html) of the correct type.
    - Cons:
        - Need to repeat relevant using declarations for every new class I create.
        - Introduces possibility for error on my side (although I usually copy and paste these).
    - Notes:
        - An alternative could be defining wrapper classes that include all the relevant types given certain template parameters??
- Public constant data members where it makes sense; standard encapsulation elsewhere.
    - Reasoning:
        - Eliminate the need for trivial getters ([C.131](https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c131-avoid-trivial-getters-and-setters)).
- Deleted default constructor for most self-defined types.
    - Reasoning:
        - Eigen matrices are default initialized to undefined values, so it's important to make sure every Eigen class data member is properly initialized. Each class' set of constructors was carefully designed to force users to explicitly define what they want.
    - Notes:
        - Might make some synthesized copy-control members dysfunctional?
- No explicitly defined copy-control members unless needed.
    - Reasoning:
        - Rule of Zero.
    - TODO:
        - investigate what happens under the hood for all types that I don't explicitly define copy-control for.
- Many of my functions that aren't part of a class return by value.
    - Reasoning:
        - More guaranteed behavior.
        - Rely on compiler to make optimizations where appropriate.
    - Notes
        - This could result in expensive copy operations.
        - An alternative is returning by reference, but undefined behavior would result if object to be returned is defined and created locally (in the function itself). Returning by reference would require the user to do something like manually define their object first, then pass it by reference as an argument to the function.
        - Another alternative is returning a smart pointer (like a unique_ptr factory), but this would also require the user to do more work.
    - TODO:
        - read up on RVO to see if this is really a problem at all.
            - i think i understand copy elision eliminating the copy action into the temporary, but it seems like there's still a copy-assign happening at the call site. when a function returns a local object by value, is there a way to just directly use move semantics? or does the compiler do that implicitly?
        - explore more alternate possibilities to avoid expensive copy operation while maintaining ease of use.
- TODO: look into implementing some kind of "verbose" user option that would enable diagnostic prints.
- TODO: look into more methods of reducing compile times.


## File-by-file implementation details

### lti/

#### lti/plant.hpp
*Plant*
- *namespace dynamical::lti::sim*
- *type: abstract class template*
- overall notes
    - Design with inheritance was used despite there only being two children of Plant. This is because there are several instances where the distinction between discrete-time and continuous-time plants are ignored (e.g. controllability and observability), but there are also significant differences between the two (discrete-time systems are characterized by difference equations whereas continuous-time plants are characterized by differential equations).
- constructor
    - deleted default constructor
        - In general, it doesn't make sense to create a Plant without any dynamics.
        - Note that if users do need to create a Plant without any dynamics, they can still do so explicitly.
    - one defined constructor, with the initial state vector as the first argument and default arguments for the C and D matrices.
        - In EECS16B, the concept of output is not taught and the C and D matrices are ignored. It is assumed that all states are outputs (x_vec = y_vec, so C = Identity) and that there is no feedthrough (D = Zero).
            - This is also why the template parameter num_outputs has a default argument of num_states.
        - It is more common to need to specify a nonzero initial state vector x than to define D explicitly. I also wanted to keep the ABCD matrices together for more intuitive instantiation (as opposed to sandwiching like A,B,x,C,D). As a result, the initial state vector goes as the first argument.
            - It should be noted that this opens up opportunity for error when initializing a Plant type implementation (i.e. trying to initialize a DiscretePlant with arguments ABCD, forgetting that the A will become x, B will become A, and so forth). I'm fairly certain the compiler would throw an error if this does happen, but I haven't tested this.
- gaussian noise generator
    - [Eigen docs for *NullaryExpr*](https://eigen.tuxfamily.org/dox/classEigen_1_1DenseBase.html#a57fed90701b138c1d38ec52871f4cef1)
    - The current implementation is a pair of unique_ptrs to both the generator (std::mt19937) and the distribution (std::normal_distribution<>). The presence of unique_ptrs is what forced me to define the copy-constructor and copy-assign member.
    - An alternative might be to define another type purely for random generation of Eigen matrices with more controlled / complex distributions than the one given by Random()?

*DiscretePlant*
- *namespace dynamical::lti::sim*
- *type: class template implementation of Plant*
- overall notes:
    - The first using declaration lets us inherit all non copy-control constructors directly from Plant.
    - Inheritance can be a little bit tricky with templates, see quick explanation [here](https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members) for why we need an explicit *this->* and typename declarations.

*ContinuousPlant*
- *namespace dynamical::lti::sim*
- *type: class template implementation of Plant*
- TODO:
    - check if synthesizing a lambda on every call to UpdateSim() significantly hurts performance or if it's better to just define a separate function representing the differential equations and point to it.
    - implement Update() as real-time update?
        - std::chrono
            - https://en.cppreference.com/w/cpp/chrono/duration
            - https://en.cppreference.com/w/cpp/chrono/time_point


#### lti/controller.hpp

*Feedback*
- *namespace dynamical::lti*
- *type: class template*

*Observer*
- *namespace dynamical::lti*
- *type: class template*
- overall notes:
    - **This is where the EECS16B training wheels come off!** No more default arguments that help users ignore the concept of "output" being different from "state".
    - An observer conceptually contains an internal model of the plant it's observing. The observer naturally shouldn't have the power to directly "update" the actual plant's current state *x*. Instead the observer keeps its own internal estimate of the state *x_hat* and updates that.
        - Note that in the current implementation, *x_hat* represents an estimation of the state one step ahead of the last information passed in.
    - The observer introduces a new term (*L(y - Cx_hat)*) to the original propagation equations Ax + Bu. We need to differentiate between observers for discrete-time systems and continuous-time systems because the propogation equations are different types (discrete difference equation vs continuous differential equation).
        - I couldn't think of any instances where the distinction between discrete-time and continuous-time observers can be ignored, so inheritance didn't seem appropriate. An enum seems like the easiest way to keep track of what type of system the observer is dealing with.
        - However, at the moment, I'm not sure about the practicality of actually using a complete state feedback controller based in continuous-time (numerical integration / time sync errors add up), so the current implementation is just for discrete time.
- constructor
    - The order of parameters in Observer's constructors is mostly influenced by the order of Plant's constructor. It can be argued that there are situations where users wouldn't specify an initial state estimate (in which case a default argument of zeros would be appropriate), but at this point users should understand they can just pass in *EigenMatrixType::Zero()*.

*Controller*
- *namespace dynamical::lti*
- *type: class template*


#### lti/analysis.hpp

*get_controllability_matrix*
- *namespace dynamical::lti::analysis*
- *type: function template*
- TODO:
    - I'm not entirely sure what goes on behind the scenes when I use a template argument (*state_dim*) directly as a default argument for the *num_steps* parameter, but tests have shown that this works.
    - understand this better.

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
- TODO:
    - overload with an implementation that doesn't rely on an invertible eigenbasis?


### trajectory/

#### trajectory/trajectory.hpp
- *namespace dynamical::trajectory*
- *type: abstract class template*

#### trajectory/min_energy.hpp
- *namespace dynamical::trajectory*
- *type: class template*
- TODO:
    - Not reliable at the moment! Need to figure out if I implemented soemthing wrong or if this method just doesn't play well with certain system characteristics.
