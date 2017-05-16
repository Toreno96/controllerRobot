#include <cassert>
#include <iostream>
#include "Helpers/speed.h"


int main() {
  using namespace controller;
  using tSpeedInterval = Speed< tSpeedUnitInterval >;
  using tSpeedDynamixel = Speed< tSpeedUnitDynamixel >;

  tSpeedInterval a1( tSpeedInterval::full() );
  assert( a1.val == double( 1.0 ) );
  tSpeedDynamixel a2( tSpeedDynamixel::full() );
  assert( a2.val == uint32_t( 1023 ) );
  assert( tSpeedDynamixel( 42 ).val == 42 );

  tSpeedDynamixel a3 = a1;
  std::cout << "a3.val = " << a3.val << '\n';
  tSpeedDynamixel a4 = a2;
  std::cout << "a4.val = " << a4.val << '\n';

  tSpeedInterval a5 = a1;
  std::cout << "a5.val = " << a5.val << '\n';
  tSpeedInterval a6 = a2;
  std::cout << "a6.val = " << a6.val << '\n';

  tSpeedDynamixel a7( tSpeedInterval( 0.5 ) );
  std::cout << "a7.val = " << a7.val << '\n';
}