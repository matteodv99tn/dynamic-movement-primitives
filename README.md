# Dynamic Movement Primitives Library

This is a C++ library that implements Dynamic movement Primitives (DMPs) in a generic fashion, so that it can be re-used in multiple contexts.
To achieve modularity (and type safety) the library relies on template meta-programming.

## Code structure

A `Dmp` object is a template object in which different dependencies are injected.
Injected data structures are:

- a [`Manifold`s](include/dmplib/manifolds/) representing the domain in which the DMP lies.


## Implementation requirements
### Manifold operations

To deal with manifold, we use a functional approach. 
For each type we want to create a manifold, it is recommended to create a separate header file:

- including the ``dmplib/manifolds/riemann_manifold.hpp`` header file
- this lines must be put inside the ``dmp::riemannmanifold`` namespace:
  - the expression
    ```cpp
    template <>
    struct tangent_space_dimension<Quaternion> { // < change quaternion
        static constexpr int value = 3; // < change this
    };
    ```
    This enables to use the ``dmp::riemannmanifold::tangent_space<>`` trait to query the proper tangent space object (i.e., an Eigen vector of according size);
  - two functions with the signature like
    ``` cpp
    Tangent_t logarithmic_map(const Domain_t& p, const Domain_t& x) const;
    Domain_t exponential_map(const Domain_t& p, const Tangent_t& x) const;
    ```
    In both signatures, the first argument is the point in the domain w.r.t. which the tangent space is constructed, while the second argument is respectively the point to be projected in the tangent space (for the logarithmic map) and the direction to move on the tangent space (for the exponential map).


## Implementation notes and code structure

$$
\begin{aligned}
    & \left.
    \begin{aligned}
        \tau \dot z &= \alpha_z \big(\beta_z (g-y) - z\big) + f(x) \\
        \tau \dot y &= z \\
    \end{aligned}
    \quad \right\\} && \textrm{: transformation system} \\
    & \tau \dot x = \alpha_x x && \textrm{: canonical system}
\end{aligned}
$$

where $f(x)$ is the function to be learned. With this premise, the following object (and thus dependencies) should be defined:

- `Manifold`: the riemann manifold where $g,y$ are defined. Have no external dependency;
- `CanonicalSystem`: an object defining the canonical system used to describe the DMP. Have no external dependency;
- `TranformationSystem`: an object defining the transformation system used to describe the DMP. Must embed knowledge of `Manifold` since it must know the operations that are performed on the given manifold;
- `LearnableFunction`: an object that embeds the learnable function $f(x)$. It must know some information of ``Manifold`` (e.g. the dimension of the tangent space)


