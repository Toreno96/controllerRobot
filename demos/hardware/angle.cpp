///
/// \file
/// \brief Strongly-typed angle (test).
/// \author Daniel Staśczak
///

#include <cassert>
#include <iostream>
#include "Board/angle.h"


int main() {
    using namespace controller;
    using tAngleRadians = Angle< tAngleUnitRadians >;
    using tAngleDynamixel = Angle< tAngleUnitDynamixel >;
    using tAngleSpi = Angle< tAngleUnitSpi >;

    tAngleRadians a1( tAngleRadians::full() );
    assert( a1.val() == double( 2.0 * M_PI ) );
    tAngleDynamixel a2( tAngleDynamixel::full() );
    assert( a2.val() == uint32_t( 4095 ) );
    assert( tAngleDynamixel( 42 ).val() == 42 );
    tAngleSpi b1( tAngleSpi::full() );
    assert( b1.val() == uint16_t( 32767 ) );
    assert( tAngleSpi( 0 ).val() == 0 );

    tAngleDynamixel a3 = a1;
    std::cout << "a3.val() = " << a3.val() << '\n';
    tAngleDynamixel a4 = a2;
    std::cout << "a4.val() = " << a4.val() << '\n';

    tAngleRadians a5 = a1;
    std::cout << "a5.val() = " << a5.val() << '\n';
    tAngleRadians a6 = a2;
    std::cout << "a6.val() = " << a6.val() << '\n';

    tAngleDynamixel a7( tAngleRadians( M_PI ) );
    std::cout << "a7.val() = " << a7.val() << '\n';

    tAngleSpi b2 = a1;
    std::cout << "b2.val() = " << b2.val() << '\n';
    tAngleSpi b3 = a2;
    std::cout << "b3.val() = " << b3.val() << '\n';

    tAngleDynamixel a8 = b1;
    std::cout << "a8.val() = " << a8.val() << '\n';
    tAngleRadians a9 = b1;
    std::cout << "a9.val() = " << a9.val() << '\n';

    tAngleSpi b4( tAngleSpi::full() / 2 );
    std::cout << "b4.val() = " << b4.val() << '\n';
}
