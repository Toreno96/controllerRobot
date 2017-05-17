#include "Helpers/current.h"

namespace controller {

template<>
constexpr uint16_t Current< tCurrentUnitDynamixel >::full() {
    return 1193;
}
template<>
constexpr double Current< tCurrentUnitInterval >::full() {
    return 1.0;
}
template<>
constexpr double Current< tCurrentUnitAmpers >::full() {
    return Current< tCurrentUnitDynamixel >::full() * 0.00269;
}

template< typename U > template< typename T >
Current< U >::operator Current< typename std::enable_if_t<
        !std::is_same< T, tCurrentUnitDynamixel >::value,
        tCurrentUnitDynamixel > >() {
    if( std::is_same< T, tCurrentUnitInterval >::value ) {
        return Current< tCurrentUnitDynamixel >( static_cast< uint16_t >( val *
                Current< tCurrentUnitDynamixel >::full() ) );
    }
    else if( std::is_same< T, tCurrentUnitAmpers >::value ) {
        return Current< tCurrentUnitDynamixel >( static_cast< uint16_t >( val /
                ( Current< tCurrentUnitAmpers >::full() /
                Current< tCurrentUnitDynamixel >::full() ) ) );
    }
}
template< typename U > template< typename T >
Current< U >::operator Current< typename std::enable_if_t<
        !std::is_same< T, tCurrentUnitInterval >::value,
        tCurrentUnitInterval > >() {
    if( std::is_same< T, tCurrentUnitDynamixel >::value ) {
        return Current< tCurrentUnitInterval >( val /
                Current< tCurrentUnitDynamixel >::full() );
    }
    else if( std::is_same< T, tCurrentUnitAmpers >::value ) {
        return Current< tCurrentUnitInterval >( val /
                Current< tCurrentUnitAmpers >::full() );
    }
}
template< typename U > template< typename T >
Current< U >::operator Current< typename std::enable_if_t<
      !std::is_same< T, tCurrentUnitAmpers >::value,
      tCurrentUnitAmpers > >() {
    if( std::is_same< T, tCurrentUnitDynamixel >::value ) {
        return Current< tCurrentUnitAmpers >( val *
                ( Current< tCurrentUnitAmpers >::full() /
                Current< tCurrentUnitDynamixel >::full() ) );
    }
    else if( std::is_same< T, tCurrentUnitInterval >::value ) {
        return Current< tCurrentUnitAmpers >( val *
                Current< tCurrentUnitAmpers >::full() );
    }
}

} // namespace controller