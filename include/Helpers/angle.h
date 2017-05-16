#ifndef _HELPERS_ANGLE_H_
#define _HELPERS_ANGLE_H_

#include <cmath>
#include <cstdint>
#include <type_traits>

namespace controller {

template< typename T, int Radians, int Dynamixel >
struct AngleUnit {
    using value_type = T;
};

using tAngleUnitRadians = AngleUnit< double, 1, 0 >;
using tAngleUnitDynamixel = AngleUnit< uint32_t, 0, 1 >;

template< typename U >
struct Angle {
    typename U::value_type val;
    explicit Angle( typename U::value_type d ) : val( d ) {}

    template< typename T = U >
    operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitRadians >::value,
        tAngleUnitRadians > >();
    template< typename T = U >
    operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitDynamixel >::value,
        tAngleUnitDynamixel > >();

    static constexpr typename U::value_type full();
};

} // namespace controller

#include "Helpers/angle.tpp"

#endif // #ifndef _HELPERS_ANGLE_H_