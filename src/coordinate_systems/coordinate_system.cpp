#include "dmplib/coordinate_systems/coordinate_system.hpp"

using CS = dmp::CoordinateSystem;

CS::CoordinateSystem(const double& initial_value) {
    _x = initial_value;
}
