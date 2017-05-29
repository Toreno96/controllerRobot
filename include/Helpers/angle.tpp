#include "Helpers/angle.h"

namespace controller {

template<>
constexpr uint32_t Angle< tAngleUnitDynamixel >::full() {
    return 4095;
}
template<>
constexpr double Angle< tAngleUnitRadians >::full() {
    return 2.0 * M_PI;
}

template< typename U > template< typename T >
Angle< U >::operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitDynamixel >::value,
        tAngleUnitDynamixel > >() {
    if( std::is_same< T, tAngleUnitRadians >::value ) {
        return Angle< tAngleUnitDynamixel >( static_cast< uint32_t >(
                val * Angle< tAngleUnitDynamixel >::full() /
                Angle< tAngleUnitRadians>::full() ) );
    }
}
template< typename U > template< typename T >
Angle< U >::operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitRadians >::value,
        tAngleUnitRadians > >() {
    if( std::is_same< T, tAngleUnitDynamixel >::value ) {
        return Angle< tAngleUnitRadians >( val *
                Angle< tAngleUnitRadians>::full() /
                Angle< tAngleUnitDynamixel >::full() );
    }
}

} // namespace controller