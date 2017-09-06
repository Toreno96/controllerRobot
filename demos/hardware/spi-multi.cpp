// Compile with: clang++ -g -O0 -DDEBUG -std=c++14 -Wall -Wextra -Wshadow -Wconversion ./demos/hardware/spi-multi.cpp ./src/Board/Galgo/spi.cpp -o ./demos/hardware/spi-multi.out -I . -I ./include/ -I -lftd2xx

#include <array>
#include <iomanip>
#include <iostream>
#include <string>
#include "Board/Galgo/spi.h"

namespace test {

struct Spi {
    controller::d2xxwrapper::Spi device;
    unsigned validDataCount;
};

} // namespace test

int main() {
    try {
        using namespace controller::d2xxwrapper;
        std::array<test::Spi, 2> spiDevices{{
                {Spi({0, 15'000, 0, 0}), 0},
                {Spi({1, 15'000, 0, 0}), 0}}};
        unsigned transmissionsCount = 1'000;
        for (unsigned i = 0; i < transmissionsCount; i++) {
            Spi::Bytes writtenBytes{0xAA};
            writtenBytes.insert(writtenBytes.end(), 9, 0xFF);
            for (auto& spi : spiDevices) {
                Spi::Bytes receivedBytes = spi.device.transfer(writtenBytes);
                int angle = (receivedBytes[2] & 0x7F) << 8 |
                    (receivedBytes[3] & 0xFF);
                int negatedAngle = (receivedBytes[4] & 0x7F) << 8 |
                    (receivedBytes[5] & 0xFF);
                if (angle == (~negatedAngle & 0x7FFF)) {
                    std::cout << std::setw(5) << angle <<
                            (&spi == &spiDevices[0] ? ' ' : '\n');
                    ++spi.validDataCount;
                }
            }
        }
        int spiDeviceNumber = 1;
        for (auto& spi : spiDevices) {
            std::clog << '#' << spiDeviceNumber << " valid data: " <<
                spi.validDataCount << '\n'
                << '#' << spiDeviceNumber << " invalid data: " <<
                transmissionsCount - spi.validDataCount << '\n';
            ++spiDeviceNumber;
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

