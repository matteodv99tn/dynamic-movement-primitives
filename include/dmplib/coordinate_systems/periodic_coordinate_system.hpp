#ifndef DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP__
#define DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP__

#include "dmplib/coordinate_systems/coordinate_system.hpp"

namespace dmp {

class PeriodicCs : public CoordinateSystem<PeriodicCs, PERIODIC> {
public:
    PeriodicCs();

    double get_Omega() const;

protected:
    // CRTP traits definition
    friend class Integrable<PeriodicCs>;
    void step_impl();
};

}  // namespace dmp


#endif  // DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP__
