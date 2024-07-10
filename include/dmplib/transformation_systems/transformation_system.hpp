#ifndef DMPLIB_TRANSFORMATION_SYSTEM_HPP
#define DMPLIB_TRANSFORMATION_SYSTEM_HPP

#include <cstdlib>
#include <iostream>

#include "dmplib/class_traits/integrable.hpp"
#include "dmplib/manifolds/concepts.hpp"
#include "dmplib/manifolds/rn_manifold.hpp"
#include "dmplib/time_axis.hpp"
#include "fmt/ostream.h"

namespace dmp::transformationsystem {

template <typename Der, dmp::riemannmanifold::concepts::riemann_manifold M>
class TransformationSystem : public Integrable<TransformationSystem<Der, M>> {
public:
    using Domain_t  = M;
    using Tangent_t = dmp::riemannmanifold::tangent_space_t<Domain_t>;

    TransformationSystem(::dmp::TimeAxis::Reference time_axis) :
            Integrable<TransformationSystem<Der, M>>(time_axis),
            _y(dmp::riemannmanifold::default_constructor<Domain_t>()),   // NOLINT
            _y0(dmp::riemannmanifold::default_constructor<Domain_t>()),  // NOLINT
            _g(dmp::riemannmanifold::default_constructor<Domain_t>())    // NOLINT
    {};

    void
    set_initial_pos_state(const Domain_t& pos) {
        _y0 = pos;
    }

    void
    set_pos_goal_state(const Domain_t& pos) {
        _y0 = pos;
    }

    void
    set_pos_state(const Domain_t& pos) {
        _y = pos;
    }

    [[nodiscard]] double
    get_period() const {
        return Integrable<TransformationSystem<Der, M>>::T();
    }

    [[nodiscard]] Tangent_t
    get_forcing_term() const {
        return _f;
    }

    void
    set_forcing_term(const Tangent_t& f) const {
        _f = f;
    }

protected:
    [[nodiscard]] Tangent_t
    delta_pos_gain() const {
        auto gain = dmp::riemannmanifold::logarithmic_map(_g, _y0);
        for (const double& v : gain) {
            if (std::abs(v) < 1e-3)  // NOLINT: magic number
                fmt::println(std::cerr, "Gain is small ({})", v);
        }
        return gain;
    }

    Domain_t  _y;   // current position state
    Domain_t  _y0;  // initial position state
    Domain_t  _g;   // goal
    Tangent_t _f;   // forcing term
};


}  // namespace dmp::transformationsystem

/*
namespace dmp {

template <typename Derived, typename Manifold>
class TransformationSystem : public Integrable<Derived> {
public:
    using M                = Manifold;  // NOLINT
    using Domain_t         = typename M::Domain_t;
    using Tangent_t        = typename M::Tangent_t;
    using ConstdoubleRef_t = std::reference_wrapper<const double>;


    using Forcing_t = typename M::Tangent_t;

protected:
    Manifold         _M;  // NOLINT
    Forcing_t        _f;
    ConstdoubleRef_t _T;  // NOLINT

    Domain_t _y;  //< "position" state
    Domain_t _g;  //< goal

public:
    TransformationSystem(const ConstdoubleRef_t& T) :
            _T(T),
            _f(Forcing_t::Zero()),
            _y(_M.construct_domain()),
            _g(_M.construct_domain()) {}

    inline Forcing_t
    get_forcing_term() const {
        return _f;
    }

    template <typename T>
    Forcing_t
    forcing_term_from_demonstration(const T& sample) {
        return static_cast<Derived*>(this)->forcing_term_from_demonstration_impl(sample
        );
    }

    template <typename T>
    std::vector<Forcing_t>
    forcing_term_from_demonstration(const std::vector<T>& trajectory) {
        namespace rv = ranges::views;
        namespace rs = ranges;
        return trajectory | rv::transform([this](const auto& sample) {
                   return forcing_term_from_demonstration(sample);
               })
               | rs::to_vector;
    }

    inline void
    set_forcing_term(const Forcing_t& f) {
        _f = f;
    }

    inline Domain_t
    get_state() const {
        return _y;
    }

    inline void
    set_state(const Domain_t& y) {
        _y = y;
    }

    inline Domain_t
    get_goal() const {
        return _g;
    }

    inline void
    set_goal(const Domain_t& g) {
        _g = g;
    }
};
}  // namespace dmp
*/


#endif  // DMPLIB_TRANSFORMATION_SYSTEM_HPP
