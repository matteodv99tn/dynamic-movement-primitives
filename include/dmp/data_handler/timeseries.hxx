#ifndef DMPLIB_TIMESERIES_HXX__
#define DMPLIB_TIMESERIES_HXX__

#include <cstddef>
#include <type_traits>

#include "dmp/class_traits/integrable.hpp"
#include "dmp/data_handler/data_serialiser.hpp"
#include "dmp/data_handler/timeseries.hpp"
#include "range/v3/all.hpp"


using dmp::TimeSeriesContainer;

template <typename T>
TimeSeriesContainer<T>::TimeSeriesContainer() {
    dmp::Integrable i;
    _dt = i.get_integration_period();
    _t0 = 0;
}

template <typename T>
TimeSeriesContainer<T>::TimeSeriesContainer(const Integrable* const integrable_obj) {
    _dt = integrable_obj->get_integration_period();
    _t0 = 0;
}

template <typename T>
TimeSeriesContainer<T>::TimeSeriesContainer(const double dt, const double t0) {
    _dt = dt;
    _t0 = t0;
}

template <typename T>
void
TimeSeriesContainer<T>::set_time_shift(const double t0) {
    _t0 = t0;
}

template <typename T>
void
TimeSeriesContainer<T>::push_back(const T& entry) {
    _data.push_back(entry);
}

template <typename T>
std::size_t
TimeSeriesContainer<T>::size() const {
    return _data.size();
}

template <typename T>
Eigen::MatrixXd
TimeSeriesContainer<T>::to_table() const {
    const int       tab_size = static_cast<int>(size());
    Eigen::MatrixXd table(tab_size, Serialiser::n_entries);

    for (auto i : ranges::views::iota(0, tab_size))
        table.row(i) = Serialiser::serialise_data(_data[i]);

    return table;
}

template <typename T>
Eigen::VectorXd
TimeSeriesContainer<T>::get_time_axis() const {
    return Eigen::VectorXd::LinSpaced(size(), _t0, _t0 + (size() - 1) * _dt);
}

template <typename T>
Eigen::VectorXi
TimeSeriesContainer<T>::get_time_indexes() const {
    return Eigen::VectorXi::LinSpaced(size(), 0, size());
}

#endif  // DMPLIB_TIMESERIES_HXX__
