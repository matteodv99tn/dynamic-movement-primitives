#ifndef DMPLIB_DATA_CONTAINER_HPP__
#define DMPLIB_DATA_CONTAINER_HPP__

#include <cstddef>
#include <Eigen/Dense>
#include <vector>

#include "dmp/class_traits/integrable.hpp"
#include "dmp/data_handler/data_serialiser.hpp"

namespace dmp {

template <typename T>
class TimeSeriesContainer {
private:
    std::vector<T> _data;  //< Data container
    double         _dt;    //< Integration period
    double         _t0;    //< Initial time offset
    using Serialiser = DataSerialiser<T>;


public:
    TimeSeriesContainer();

    TimeSeriesContainer(const Integrable* const integrable_obj);

    TimeSeriesContainer(const double dt, const double t0 = 0);

    void set_time_shift(const double t0);

    void push_back(const T& entry);

    std::size_t size() const;

    Eigen::MatrixXd to_table() const;

    Eigen::VectorXd get_time_axis() const;

    Eigen::VectorXi get_time_indexes() const;
};

}  // namespace dmp

#include "dmp/data_handler/timeseries.hxx"

#endif  // DMPLIB_DATA_CONTAINER_HPP__
