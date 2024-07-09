#ifndef DMPLIB_FILTERS_HPP
#define DMPLIB_FILTERS_HPP

#include "dmplib/manifolds/aliases.hpp"
#include "dmplib/manifolds/concepts.hpp"
#include "range/v3/iterator/operations.hpp"
#include "range/v3/numeric/accumulate.hpp"
#include "range/v3/range/conversion.hpp"
#include "range/v3/view/concat.hpp"
#include "range/v3/view/drop.hpp"
#include "range/v3/view/reverse.hpp"
#include "range/v3/view/transform.hpp"
#include "range/v3/view/unique.hpp"
#include "range/v3/view/zip.hpp"

namespace dmp::utils {

namespace rs = ::ranges;
namespace rv = ::ranges::views;

template <dmp::riemannmanifold::concepts::riemann_manifold T>
dmp::StampedPosTrajectory_t<T>
remove_duplicates(const dmp::StampedPosTrajectory_t<T>& original_traj) {
    return original_traj | rv::unique([](const auto& a, const auto& b) {
               return std::get<1>(a) == std::get<1>(b);
           })
           | rs::to<dmp::StampedPosTrajectory_t<T>>;
}

template <rs::forward_range RNG, rs::bidirectional_range WINDOW_RNG>
std::vector<double>
rolling_mean(const RNG& rng, const WINDOW_RNG& window) {
    // Create symmetric view of the window
    std::size_t half_window_size = rs::distance(window);
    auto        symm_window = rv::concat(rv::reverse(window | rv::drop(1)), window);

    std::vector<double> res(rs::distance(rng));
    for (std::size_t i = 0; i < res.size(); ++i) {
        auto subwind =
                symm_window | rv::drop(std::max(int(half_window_size - 1 - i), 0));
        auto zipped_view =
                rv::zip(rng | rv::drop(std::max(int(i - half_window_size + 1), 0)),
                        subwind);
        double rng_sum = rs::accumulate(
                zipped_view | rv::transform([](const auto& p) -> double {
                    return rs::get<0>(p) * rs::get<1>(p);
                }),
                0.0
        );
        double w_sum = rs::accumulate(
                zipped_view | rv::transform([](const auto& p) -> double {
                    return rs::get<1>(p);
                }),
                0.0
        );
        res[i] = rng_sum / w_sum;
    }

    return res;
}

}  // namespace dmp::utils


#endif  // DMPLIB_FILTERS_HPP
