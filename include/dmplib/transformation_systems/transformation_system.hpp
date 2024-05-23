#ifndef DMPLIB_TRANSFORMATION_SYSTEM_HPP
#define DMPLIB_TRANSFORMATION_SYSTEM_HPP

#include <functional>

#include "dmplib/class_traits/integrable.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"

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
    Manifold _M; // NOLINT
                 //
    Forcing_t        _f;
    ConstdoubleRef_t _T; // NOLINT

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


#endif  // DMPLIB_TRANSFORMATION_SYSTEM_HPP
