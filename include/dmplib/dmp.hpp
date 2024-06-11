#ifndef DMPLIB_DMP_HPP
#define DMPLIB_DMP_HPP

namespace dmp {

template <
        typename Manifold,
        typename Coordinate_System,
        typename Transformation_System,
        typename Learnable_Function>
class Dmp {
public:
    using Manifold_t             = Manifold;
    using CoordinateSystem_t     = Coordinate_System;
    using TransformationSystem_t = Transformation_System;
    using LearnableFunction_t    = Learnable_Function;


private:
    Manifold               _man;
    CoordinateSystem_t     _cs;
    TransformationSystem_t _ts;
    LearnableFunction_t    _fun;
};
}  // namespace dmp


#endif  // DMPLIB_DMP_HPP
