#ifndef DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP__
#define DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP__

#include "dmplib/coordinate_systems/coordinate_system.hpp"

namespace dmp {

class PeriodicCs: public CoordinateSystem {
public:
    static constexpr RepresentationType Type = PERIODIC;

public:
    PeriodicCs();

    void step();

    double get_Omega() const;
};

}  // namespace dmp


#endif  // DMPLIB_PERIODIC_COORDINATE_SYSTEM_HPP__
