#ifndef _WRAPPERS_D2XX_SPI_H_
#define _WRAPPERS_D2XX_SPI_H_

#include <cstdint>
#include <vector>
#include "../../../3rdParty/d2xx/include/ftd2xx.h"

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
        Spi(const Config& config);
        ~Spi();
        Bytes read(DWORD bytesCount);
        void write(const Bytes& bytes);
        Bytes transfer(const Bytes& bytes);
    private:
        void ftdiWrite(const Bytes& bytes);
        void initializeMpsse();
        void disableBy5ClockDivider();
        void setClockDivisor();
        void checkMpsseOperability();
        void disconnectLoopbackMode();
        void initializePins();
        void setChipSelect(bool state);
        const Config config_;
        FT_HANDLE ftHandle_;
};

} // namespace controller

} // namespace d2xxwrapper

#endif // #ifndef _WRAPPERS_D2XX_SPI_H_
