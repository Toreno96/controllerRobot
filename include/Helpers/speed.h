///
/// \file
/// \brief Strongly-typed speed.
/// \author Daniel Staśczak
/// \author Przemysław Walkowiak
///

#ifndef _HELPERS_SPEED_H_
#define _HELPERS_SPEED_H_

#include <cstdint>
#include <type_traits>

namespace controller {

template< typename T, int Dynamixel, int Interval >
struct SpeedUnit {
    using value_type = T;
};

using tSpeedUnitDynamixel = SpeedUnit< uint32_t, 1, 0 >;
using tSpeedUnitInterval = SpeedUnit< double, 0, 1 >;

template< typename U >
class Speed {
    private:
        typename U::value_type val_;
    public:
        explicit Speed( typename U::value_type d ) : val_( d ) {}
        typename U::value_type val() {
            return val_;
        }

        template< typename T = U >
        operator Speed< typename std::enable_if_t<
                !std::is_same< T, tSpeedUnitDynamixel >::value,
                tSpeedUnitDynamixel > >();
        template< typename T = U >
        operator Speed< typename std::enable_if_t<
                !std::is_same< T, tSpeedUnitInterval >::value,
                tSpeedUnitInterval > >();

        static constexpr typename U::value_type full();
};

} // namespace controller

#include "Helpers/speed.tpp"

#endif // #ifndef _HELPERS_SPEED_H_