///
/// \file
/// \brief Wrapper for SPI communication using D2XX library (test for Spi instances as elements in map).
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

// Compile with: clang++ -g -O0 -DDEBUG -std=c++14 -Wall -Wextra -Wshadow -Wconversion ./demos/hardware/spi-map.cpp ./src/Board/Galgo/spi.cpp -o ./demos/hardware/spi-map.out -I . -I ./include/ -lftd2xx

#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include "Board/Galgo/spi.h"

using controller::d2xxwrapper::Spi;

uint16_t readSpiPosition(Spi& spi) {
    Spi::Bytes writtenBytes{0xAA};
    writtenBytes.insert(writtenBytes.end(), 9, 0xFF);
    const auto receivedBytes = spi.transfer(writtenBytes);
    using SizeType = decltype(receivedBytes)::size_type;
    auto converter =
        [&receivedBytes](SizeType firstBytePosition, SizeType secondBytePosition) {
            return (receivedBytes[firstBytePosition] & 0x7F) << 8 |
                (receivedBytes[secondBytePosition] & 0xFF);
        };
    auto data = converter(2, 3);
    auto negatedData = converter(4, 5);
    if (data == (~negatedData & 0x7FFF))
        return static_cast<uint16_t>(data);
    else
        throw std::runtime_error("Data read from SPI is invalid");
}

int main() {
    try {
        std::map<int, Spi> spiDevices;
        spiDevices.emplace(1, Spi({0, 15'000, 0, 0}));
        spiDevices.emplace(2, Spi({1, 15'000, 0, 0}));
        unsigned transmissionsCount = 1000;
        for (unsigned i = 0; i < transmissionsCount; ++i) {
            std::clog << std::setw(5) << readSpiPosition(spiDevices.at(1)) << ' '
                    << std::setw(5) << readSpiPosition(spiDevices.at(2)) << '\n';

        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
}

