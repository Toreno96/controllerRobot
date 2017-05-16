#include "Helpers/speed.h"

namespace controller {

template<>
constexpr double Speed< tSpeedUnitInterval >::full() {
    return 1.0;
}
template<>
constexpr uint32_t Speed< tSpeedUnitDynamixel >::full() {
    return 1023;
}

template< typename U > template< typename T >
Speed< U >::operator Speed< typename std::enable_if_t<
        !std::is_same< T, tSpeedUnitInterval >::value,
        tSpeedUnitInterval > >() {
    if( std::is_same< T, tSpeedUnitDynamixel >::value ) {
        return Speed< tSpeedUnitInterval >( val /
                Speed< tSpeedUnitDynamixel >::full() );
    }
}
template< typename U > template< typename T >
Speed< U >::operator Speed< typename std::enable_if_t<
        !std::is_same< T, tSpeedUnitDynamixel >::value,
        tSpeedUnitDynamixel > >() {
    if( std::is_same< T, tSpeedUnitInterval >::value ) {
        return Speed< tSpeedUnitDynamixel >( static_cast< uint32_t >(
                val * Speed< tSpeedUnitDynamixel >::full() ) );
    }
}

} // namespace controller