#include "Wrappers/d2xx/spi.h"

namespace controller {

namespace d2xxwrapper {


Spi::Spi(const Config& config) : config_(config) {}
Spi::~Spi() {}
Spi::Bytes Spi::read(DWORD bytesCount) {}
void Spi::write(const Spi::Bytes& bytes) {}
Spi::Bytes Spi::transfer(const Spi::Bytes& bytes) {}
void Spi::initializeMpsse() {}
void Spi::disableBy5ClockDivider() {}
void Spi::setClockDivisor() {}
void Spi::checkMpsseOperability() {}
void Spi::disconnectLoopbackMode() {}
void Spi::initializePins() {}
void Spi::setChipSelect(bool state) {}

} // namespace controller

} // namespace d2xxwrapper
