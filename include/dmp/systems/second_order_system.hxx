#ifndef DMPLIB_SECOND_ORDER_SYSTEM_HXX__
#define DMPLIB_SECOND_ORDER_SYSTEM_HXX__

namespace dmp {

template <typename M>
SecondOrderSystem<M>::SecondOrderSystem() {
    alpha = 48.0;
    beta  = alpha / 4.0;
    g     = M::construct_domain();
}

}  // namespace dmp


#endif  // DMPLIB_SECOND_ORDER_SYSTEM_HXX__
