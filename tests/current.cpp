#include <cassert>
#include <iostream>
#include "Helpers/current.h"


int main() {
    using namespace controller;
    using tCurrentAmpers = Current< tCurrentUnitAmpers >;
    using tCurrentInterval = Current< tCurrentUnitInterval >;
    using tCurrentDynamixel = Current< tCurrentUnitDynamixel >;

    tCurrentAmpers a1( tCurrentAmpers::full() );
    std::cout << "a1.val() = " << a1.val() << '\n';
    tCurrentInterval a2( tCurrentInterval::full() );
    assert( a2.val() == double( 1.0 ) );
    tCurrentDynamixel a3( tCurrentDynamixel::full() );
    assert( a3.val() == uint32_t( 1193 ) );
    assert( tCurrentDynamixel( 42 ).val() == 42 );

    tCurrentDynamixel a4 = a1;
    std::cout << "a4.val() = " << a4.val() << '\n';
    tCurrentDynamixel a5 = a2;
    std::cout << "a5.val() = " << a5.val() << '\n';

    tCurrentInterval a6 = a1;
    std::cout << "a6.val() = " << a6.val() << '\n';
    tCurrentInterval a7 = a2;
    std::cout << "a7.val() = " << a7.val() << '\n';

    tCurrentDynamixel a8( tCurrentInterval( 0.5 ) );
    std::cout << "a8.val() = " << a8.val() << '\n';
}