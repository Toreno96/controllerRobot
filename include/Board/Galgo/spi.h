///
/// \file
/// \brief Wrapper for SPI communication using D2XX library.
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

#ifndef _WRAPPERS_D2XX_SPI_H_
#define _WRAPPERS_D2XX_SPI_H_

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
#include "3rdParty/d2xx/include/ftd2xx.h"

namespace controller {

namespace d2xxwrapper {

class Spi {
    public:
        struct Config {
            const int port;
            const int frequency;
            const DWORD readTimeout;
            const DWORD writeTimeout;
        };
        using Byte = uint8_t;
        using Bytes = std::vector<Byte>;
        Spi() = delete;
        Spi(const Config& config);
        ~Spi();
        Spi(const Spi&) = delete;
        Spi(Spi&& other);
        Spi& operator=(const Spi&) = delete;
        Spi& operator=(Spi&& other);
        Bytes read(DWORD bytesCount);
        void write(const Bytes& bytes);
        Bytes transfer(const Bytes& bytes);
    private:
        void ftdiWrite(const Bytes& bytes);
        void initializeMpsse(const Config& config);
        void disableBy5ClockDivider();
        void setClockDivisor(const Config& config);
        void checkMpsseOperability();
        void disconnectLoopbackMode();
        void initializePins();
        void setChipSelect(bool state);
        int port_;
        FT_HANDLE ftHandle_;
        static std::unordered_set<int> usedPorts_;
};


class SpiOpenPortException: public std::runtime_error{
public:
    SpiOpenPortException();
    SpiOpenPortException(int port);
    SpiOpenPortException(int port, FT_STATUS status);
};

class SpiReadException: public std::runtime_error{
public:
    SpiReadException();
    SpiReadException(FT_STATUS status);
    SpiReadException(int attempts, FT_STATUS status);
};

class SpiWriteException: public std::runtime_error{
public:
    SpiWriteException();
    SpiWriteException(FT_STATUS status);
};

class MpsseFailedException: public std::runtime_error{
public:
    MpsseFailedException();
    MpsseFailedException(const std::string& description);
};

} // namespace controller

} // namespace d2xxwrapper

#endif // #ifndef _WRAPPERS_D2XX_SPI_H_
