#include "Wrappers/d2xx/spi.h"
#include <cmath>

namespace controller {

namespace d2xxwrapper {


Spi::Spi(const Config& config) : config_(config) {}

Spi::~Spi() {}

Spi::Bytes Spi::read(DWORD bytesCount) {}

void Spi::write(const Spi::Bytes& bytes) {}

Spi::Bytes Spi::transfer(const Spi::Bytes& bytes) {}

void Spi::initializeMpsse() {
	FT_ResetDevice(ftHandle_);
    FT_SetBitMode(ftHandle_, 0xBB, 2);
    FT_SetLatencyTimer(ftHandle_, 16);
    FT_SetTimeouts(ftHandle_, config_.readTimeout, config_.writeTimeout);
    
    disableBy5ClockDivider();
    setClockDivisor();
    checkMpsseOperability();
}

void Spi::disableBy5ClockDivider() {
	write({0x8A});
}

void Spi::setClockDivisor() {
    int divisor = int(ceil((30000000.0 - double(config_.frequency)) /
    	double(config_.frequency))) & 0xFFFF;
	write({0x86, uint8_t(divisor & 0xFF), uint8_t((divisor >> 8) & 0xFF)});
}

void Spi::checkMpsseOperability() {
	uint8_t commands[2] = {0xAA, 0xAB};
	Bytes answer;
	
	for (int i = 0; i < 2; ++i) {
		write({commands[i]});
		answer = read(2);
		
        if (answer[0] != 0xFA || answer[1] != commands[i]) {
        	FT_Close(ftHandle_);
        	throw std::runtime_error("MPSSE did not respond correctly");
        }
	}
}

void Spi::disconnectLoopbackMode() {
	write({0x85});
}

void Spi::initializePins() {
	write({0x80, 0x22, 0x23});
}

void Spi::setChipSelect(bool state) {
	write({0x80, static_cast<uint8_t>(0x02 | ((state & 1) << 5)), 0x23});
}

} // namespace controller

} // namespace d2xxwrapper
