// Compile with: clang++ -g -O0 -DDEBUG -std=c++14 -Wall -Wextra -Wshadow -Wconversion ./tests/spi-ports.cpp ./src/Wrappers/d2xx/spi.cpp -o ./tests/spi-ports.out -I ./include/ -lftd2xx

#include <iostream>
#include "Wrappers/d2xx/spi.h"

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
