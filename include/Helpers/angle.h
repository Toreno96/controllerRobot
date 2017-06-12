///
/// \file
/// \brief Strongly-typed angle.
/// \author Daniel Staśczak
/// \author Przemysław Walkowiak
///

#ifndef _HELPERS_ANGLE_H_
#define _HELPERS_ANGLE_H_

#include <cmath>
#include <cstdint>
#include <type_traits>

namespace controller {

template< typename T, int Dynamixel, int Radians >
struct AngleUnit {
    using value_type = T;
};

using tAngleUnitDynamixel = AngleUnit< uint32_t, 1, 0 >;
using tAngleUnitRadians = AngleUnit< double, 0, 1 >;

template< typename U >
class Angle {
    private:
        typename U::value_type val_;
    public:
        explicit Angle( typename U::value_type d ) : val_( d ) {}
        typename U::value_type val() {
            return val_;
        }

        template< typename T = U >
        operator Angle< typename std::enable_if_t<
                !std::is_same< T, tAngleUnitDynamixel >::value,
                tAngleUnitDynamixel > >();
        template< typename T = U >
        operator Angle< typename std::enable_if_t<
                !std::is_same< T, tAngleUnitRadians >::value,
                tAngleUnitRadians > >();

        static constexpr typename U::value_type full();
};

} // namespace controller

#include "Helpers/angle.tpp"

#endif // #ifndef _HELPERS_ANGLE_H_