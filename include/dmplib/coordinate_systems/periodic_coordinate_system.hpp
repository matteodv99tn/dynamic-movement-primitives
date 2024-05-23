#ifndef DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP
#define DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP

#include "dmplib/coordinate_systems/coordinate_system.hpp"

namespace dmp {

class PeriodicCs : public CoordinateSystem<PeriodicCs, PERIODIC> {
public:
    PeriodicCs();

    [[nodiscard]] double get_Omega(  // NOLINT: desired to have Omega with capital "O"
    ) const;

protected:
    // CRTP traits definition
    friend class Integrable<PeriodicCs>;
    void step_impl();
};

}  // namespace dmp


#endif  // DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP
