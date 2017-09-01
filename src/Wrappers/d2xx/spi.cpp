#include <cmath>
#include <stdexcept>
#include <string>
#include "Wrappers/d2xx/spi.h"

namespace controller {

namespace d2xxwrapper {


Spi::Spi(const Config& config) : ftHandle_(nullptr) {
    FT_STATUS ftStatus = FT_Open(config.port, &ftHandle_);
    if (ftStatus != FT_OK) {
        throw std::runtime_error("FT_Open(" + std::to_string(config.port) +
                ") failed with FT_STATUS == " + std::to_string(ftStatus));
    }
    initializeMpsse(config);
    disconnectLoopbackMode();
    initializePins();
}

Spi::~Spi() {
    FT_Close(ftHandle_);
}

Spi::Bytes Spi::read(DWORD bytesCount) {
    FT_STATUS ftStatus;

    DWORD bytesInQueue = 0;
    unsigned attempts = 0;
    do {
        if (attempts++ < 10000)
            ftStatus = FT_GetQueueStatus(ftHandle_, &bytesInQueue);
        else {
            using namespace std::string_literals;
            throw std::runtime_error("Spi::read failed! "s +
                    "Too much attempts of FT_GetQueueStatus."s +
                    "The most recent FT_STATUS == "s + std::to_string(ftStatus));
        }
    }
    while (bytesInQueue < bytesCount);

    Bytes receivedBytes(bytesCount);
    DWORD receivedBytesCount = 0;
    ftStatus = FT_Read(ftHandle_, receivedBytes.data(), bytesCount,
            &receivedBytesCount);
    if (ftStatus != FT_OK) {
        throw std::runtime_error("FT_Read failed with FT_STATUS == " +
                std::to_string(ftStatus));
    }
    // Sprawdzanie czy receivedBytesCount == bytesCount?
    else
        return receivedBytes;
}

void Spi::write(const Spi::Bytes& bytes) {
    throw std::runtime_error("Not implemented yet");
    bytes.size();
}

Spi::Bytes Spi::transfer(const Spi::Bytes& bytes) {
    Bytes transferBytes{0x31, static_cast<Byte>(bytes.size() - 1), 0x00};
    transferBytes.insert(transferBytes.end(), bytes.begin(), bytes.end());
    setChipSelect(false);
    ftdiWrite(transferBytes);
    setChipSelect(true);
    return read(bytes.size());
}

void Spi::ftdiWrite(const Spi::Bytes& bytes) {
    Bytes buffer = bytes;
    DWORD bytesWritten = 0;
    FT_STATUS ftStatus = FT_Write(ftHandle_, buffer.data(), buffer.size(),
            &bytesWritten);
    if (ftStatus != FT_OK) {
        throw std::runtime_error("FT_Write failed with FT_STATUS == " +
                std::to_string(ftStatus));
    }
}

void Spi::initializeMpsse(const Config& config) {
    FT_ResetDevice(ftHandle_);
    FT_SetBitMode(ftHandle_, 0xBB, 2);
    FT_SetLatencyTimer(ftHandle_, 16);
    FT_SetTimeouts(ftHandle_, config.readTimeout, config.writeTimeout);
    disableBy5ClockDivider();
    setClockDivisor(config);
    checkMpsseOperability();
}

void Spi::disableBy5ClockDivider() {
    ftdiWrite({0x8A});
}

void Spi::setClockDivisor(const Config& config) {
    int divisor = int(ceil((30000000.0 - double(config.frequency)) /
            double(config.frequency))) & 0xFFFF;
    ftdiWrite({0x86, Byte(divisor & 0xFF), Byte((divisor >> 8) & 0xFF)});
}

void Spi::checkMpsseOperability() {
    for (auto command : {Byte(0xAA), Byte(0xAB)}) {
        ftdiWrite({command});
        Bytes answer = read(2);
        if (answer[0] != 0xFA || answer[1] != command) {
            throw std::runtime_error("MPSSE did not respond correctly");
        }
    }
}

void Spi::disconnectLoopbackMode() {
    ftdiWrite({0x85});
}

void Spi::initializePins() {
    ftdiWrite({0x80, 0x22, 0x23});
}

void Spi::setChipSelect(bool state) {
    ftdiWrite({0x80, static_cast<Byte>(0x02 | ((state & 1) << 5)), 0x23});
}

} // namespace controller

} // namespace d2xxwrapper
