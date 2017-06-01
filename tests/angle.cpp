#include <cassert>
#include <iostream>
#include "Helpers/angle.h"


int main() {
  using namespace controller;
  using tAngleRadians = Angle< tAngleUnitRadians >;
  using tAngleDynamixel = Angle< tAngleUnitDynamixel >;

  tAngleRadians a1( tAngleRadians::full() );
  assert( a1.val() == double( 2.0 * M_PI ) );
  tAngleDynamixel a2( tAngleDynamixel::full() );
  assert( a2.val() == uint32_t( 4095 ) );
  assert( tAngleDynamixel( 42 ).val() == 42 );

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
}