///
/// \file
/// \brief Wrapper for SPI communication using D2XX library (test for connecting to the same port more than one time).
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

// Compile with: clang++ -g -O0 -DDEBUG -std=c++14 -Wall -Wextra -Wshadow -Wconversion ./demos/hardware/spi-ports.cpp ./src/Board/Galgo/spi.cpp -o ./demos/hardware/spi-ports.out -I . -I ./include/ -lftd2xx

#include <iostream>
#include "Board/Galgo/spi.h"

int main() {
    try {
        using controller::d2xxwrapper::Spi;
        Spi s1({0, 15'000, 0, 0});
        Spi s2({1, 15'000, 0, 0});
        Spi s3({1, 15'000, 0, 0});
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}
