#ifndef DMPLIB_TRANSFORMATION_SYSTEM_HPP__
#define DMPLIB_TRANSFORMATION_SYSTEM_HPP__

#include <functional>

#include "dmplib/class_traits/integrable.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/transform.hpp"

namespace dmp {

template <typename Derived, typename Manifold>
class TransformationSystem : public Integrable<Derived> {
public:
    using M              = Manifold;
    using Domain_t       = typename M::Domain_t;
    using Tangent_t      = typename M::Tangent_t;
    using constdoubleRef = std::reference_wrapper<const double>;


    using Forcing_t = typename M::Tangent_t;

protected:
    Forcing_t   _f;
    constdoubleRef _T;

    Domain_t _y;  //< "position" state
    Domain_t _g;  //< goal

    Manifold _M;


public:
    TransformationSystem(const constdoubleRef& T) : _T(T) {
        _f = Forcing_t::Zero();
        _y = this->_M.construct_domain();
        _g = this->_M.construct_domain();
    }

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


#endif  // DMPLIB_TRANSFORMATION_SYSTEM_HPP__
