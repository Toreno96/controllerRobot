///
/// \file
/// \brief Strongly-typed angle.
/// \author Daniel Staśczak
/// \author Przemysław Walkowiak
///

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
template<>
constexpr uint16_t Angle< tAngleUnitSpi >::full() {
    return 32767;
}

template< typename U > template< typename T >
Angle< U >::operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitDynamixel >::value,
        tAngleUnitDynamixel > >() {
    if( std::is_same< T, tAngleUnitRadians >::value ) {
        return Angle< tAngleUnitDynamixel >( static_cast< uint32_t >(
                val_ * Angle< tAngleUnitDynamixel >::full() /
                Angle< tAngleUnitRadians>::full() ) );
    }
    else if( std::is_same< T, tAngleUnitSpi >::value ) {
        return Angle< tAngleUnitDynamixel >( static_cast< uint32_t >(
                val_ * Angle< tAngleUnitDynamixel >::full() /
                Angle< tAngleUnitSpi >::full() ) );
    }
}
template< typename U > template< typename T >
Angle< U >::operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitRadians >::value,
        tAngleUnitRadians > >() {
    if( std::is_same< T, tAngleUnitDynamixel >::value ) {
        return Angle< tAngleUnitRadians >( val_ *
                Angle< tAngleUnitRadians>::full() /
                Angle< tAngleUnitDynamixel >::full() );
    }
    else if( std::is_same< T, tAngleUnitSpi >::value ) {
        return Angle< tAngleUnitRadians >( val_ *
                Angle< tAngleUnitRadians >::full() /
                Angle< tAngleUnitSpi >::full() );
    }
}
template< typename U > template< typename T >
Angle< U >::operator Angle< typename std::enable_if_t<
        !std::is_same< T, tAngleUnitSpi >::value,
        tAngleUnitSpi > >() {
    if( std::is_same< T, tAngleUnitDynamixel >::value ) {
        return Angle< tAngleUnitSpi >( static_cast< uint16_t >(
                val_ * Angle< tAngleUnitSpi >::full() /
                Angle< tAngleUnitDynamixel>::full() ) );
    }
    else if( std::is_same< T, tAngleUnitRadians >::value ) {
        return Angle< tAngleUnitSpi >( static_cast< uint16_t >(
                val_ * Angle< tAngleUnitSpi >::full() /
                Angle< tAngleUnitRadians >::full() ) );
    }
}

} // namespace controller
