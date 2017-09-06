// Compile with: clang++ -g -O0 -DDEBUG -std=c++14 -Wall -Wextra -Wshadow -Wconversion ./demos/hardware/spi-single.cpp ./src/Board/spi.cpp -o ./demos/hardware/spi-single.out -I ./include/ -lftd2xx

#include <iostream>
#include <string>
#include "Board/spi.h"

int main() {
    using namespace controller::d2xxwrapper;
    try {
        Spi spi({0, 15'000, 0, 0});
        unsigned validData = 0;
        unsigned invalidData = 0;
        for (int i = 0; i < 1'000; i++) {
            Spi::Bytes writtenBytes{0xAA};
            writtenBytes.insert(writtenBytes.end(), 9, 0xFF);
            Spi::Bytes receivedBytes = spi.transfer(writtenBytes);
            int angle = (receivedBytes[2] & 0x7F) << 8 |
                    (receivedBytes[3] & 0xFF);
            int negatedAngle = (receivedBytes[4] & 0x7F) << 8 |
                    (receivedBytes[5] & 0xFF);
            if (angle == (~negatedAngle & 0x7FFF)) {
              std::cout << angle << '\n';
              ++validData;
            }
            else
              ++invalidData;
        }
        std::clog << "Valid data: " << validData << '\n'
            << "Invalid invalid: " << invalidData << '\n';
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}
