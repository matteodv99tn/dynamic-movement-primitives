# Dynamic Movement Primitives Library

This is a C++ library that implements Dynamic movement Primitives (DMPs) in a generic fashion, so that it can be re-used in multiple contexts.
To achieve modularity (and type safety) the library relies on template meta-programming.

## Code structure

A `Dmp` object is a template object in which different dependencies are injected.
Injected data structures are:

- a [`Manifold`s](include/dmplib/manifolds/) representing the domain in which the DMP lies.


## Implementation requirements
### Manifolds

Each manifold class must implement the following concepts:

- having a `using` statements `Domain_t, Tangent_t` to represent types to describe both domain and tangent space;
- two functions `construct_domain(), construct_tangent()` for default initialisation of domains and tangent space objects.
- two function for the computation of the logarithmic and exponential map operators with the following signatures:
  ``` cpp
  Tangent_t logarithmic_map(const Domain_t& p, const Domain_t& x) const;
  Domain_t exponential_map(const Domain_t& p, const Tangent_t& x) const;
  ```

To easen development, one can use the [`RiemannManifold`](include/dmplib/manifolds/riemann_manifolds.hpp) class in a [Curiously Recurring Template Pattern](https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern) (CRTP) to easen development.


## Implementation notes and code structure

$$
\begin{aligned}
    & \left.
    \begin{aligned}
        \tau \dot z &= \alpha_z \big(\beta_z (g-y) - z\big) + f(x) \\
        \tau \dot y &= z \\
    \end{aligned}
    \quad \right\} && \textrm{: transformation system} \\
    & \tau \dot x = \alpha_x x && \textrm{: canonical system}
\end{aligned}
$$

where $f(x)$ is the function to be learned. With this premise, the following object (and thus dependencies) should be defined:

- `Manifold`: the riemann manifold where $g,y$ are defined. Have no external dependency;
- `CanonicalSystem`: an object defining the canonical system used to describe the DMP. Have no external dependency;
- `TranformationSystem`: an object defining the transformation system used to describe the DMP. Must embed knowledge of `Manifold` since it must know the operations that are performed on the given manifold;
- `LearnableFunction`: an object that embeds the learnable function $f(x)$. It must know some information of ``Manifold`` (e.g. the dimension of the tangent space)


