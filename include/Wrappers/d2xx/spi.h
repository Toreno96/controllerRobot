#include <cstdint>
#include <map>
#include <vector>
#include "3rdParty/d2xx/include/ftd2xx.h"


class Spi {
  public:
    struct Config {
      const std::map<int, int> portsByLeg;
      const int frequency;
      const DWORD readTimeout;
      const DWORD writeTimeout;
    };
    using Bytes = std::vector<uint8_t>;
    Spi(const Config& spiConfig);
    ~Spi();
    Bytes read(DWORD bytesCount);
    void write(const Bytes& bytes);
    Bytes transfer(const Bytes& bytes);
  private:
    void initializeMpsse();
    void disableBy5ClockDivider();
    void setClockDivisor();
    void checkMpsseOperability();
    void disconnectLoopbackMode();
    void initializePins();
    void setChipSelect(bool state);
    const Config config_;
    FT_HANDLE ftHandler_;
};
