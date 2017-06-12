///
/// \file
/// \brief General-purpose helper algorithm (test).
/// \author Daniel Staśczak
///

#include <iostream>
#include <vector>
#include "Helpers/algorithm.h"

int main() {
    std::vector< int > in1{ 5, 1, 6, 2 };
    std::vector< int > in2{ 3, 7, 4, 8 };
    auto out = controller::unsortedMerge( in1, in2 );
    for( auto value : out )
        std::cout << value << ' ';
    std::cout << '\n';
}