#ifndef _HELPERS_SPEED_H_
#define _HELPERS_SPEED_H_

#include <cstdint>
#include <type_traits>

namespace controller {

template< typename T, int Interval, int Dynamixel >
struct SpeedUnit {
    using value_type = T;
};

using tSpeedUnitInterval = SpeedUnit< double, 1, 0 >;
using tSpeedUnitDynamixel = SpeedUnit< uint32_t, 0, 1 >;

template< typename U >
struct Speed {
    typename U::value_type val;
    explicit Speed( typename U::value_type d ) : val( d ) {}

    template< typename T = U >
    operator Speed< typename std::enable_if_t<
            !std::is_same< T, tSpeedUnitInterval >::value,
            tSpeedUnitInterval > >();
    template< typename T = U >
    operator Speed< typename std::enable_if_t<
            !std::is_same< T, tSpeedUnitDynamixel >::value,
            tSpeedUnitDynamixel > >();

    static constexpr typename U::value_type full();
};

} // namespace controller

#include "Helpers/speed.tpp"

#endif // #ifndef _HELPERS_SPEED_H_