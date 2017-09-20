///
/// \file
/// \brief Wrapper for SPI communication using D2XX library.
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include "Board/Galgo/spi.h"

namespace controller {

namespace d2xxwrapper {

std::unordered_set<int> Spi::usedPorts_;

Spi::Spi(const Config& config) : port_(config.port), ftHandle_(nullptr) {
    if (usedPorts_.find(port_) != usedPorts_.end()) {
        throw Spi::OpenPortException(port_);
    }
    FT_STATUS ftStatus = FT_Open(port_, &ftHandle_);
    if (ftStatus != FT_OK) {
        throw Spi::OpenPortException(port_, ftStatus);
    }
    initializeMpsse(config);
    disconnectLoopbackMode();
    initializePins();
    usedPorts_.insert(port_);
}

Spi::~Spi() {
    if (ftHandle_ != nullptr) {
        FT_Close(ftHandle_);
        usedPorts_.erase(port_);
    }
}

Spi::Spi(Spi&& other) : port_(other.port_), ftHandle_(other.ftHandle_) {
    other.ftHandle_ = nullptr;
}

Spi& Spi::operator=(Spi&& other) {
    std::swap(port_, other.port_);
    std::swap(ftHandle_, other.ftHandle_);
    return *this;
}

Spi::Bytes Spi::read(DWORD bytesCount) {
    FT_STATUS ftStatus;

    DWORD bytesInQueue = 0;
    unsigned attempts = 0;
    do {
        if (attempts++ < 10000)
            ftStatus = FT_GetQueueStatus(ftHandle_, &bytesInQueue);
        else {
            throw Spi::ReadException(attempts, ftStatus);
        }
    }
    while (bytesInQueue < bytesCount);

    Bytes receivedBytes(bytesCount);
    DWORD receivedBytesCount = 0;
    ftStatus = FT_Read(ftHandle_, receivedBytes.data(), bytesCount,
            &receivedBytesCount);
    if (ftStatus != FT_OK) {
        throw Spi::ReadException(ftStatus);
    }
    // Sprawdzanie czy receivedBytesCount == bytesCount?
    else
        return receivedBytes;
}

void Spi::write(const Spi::Bytes& bytes) {
    Bytes writeBytes{0x11, static_cast<Byte>(bytes.size() - 1), 0x00};
    writeBytes.insert(writeBytes.end(), bytes.begin(), bytes.end());
    setChipSelect(false);
    ftdiWrite(writeBytes);
    setChipSelect(true);
}

Spi::Bytes Spi::transfer(const Spi::Bytes& bytes) {
    Bytes transferBytes{0x31, static_cast<Byte>(bytes.size() - 1), 0x00};
    transferBytes.insert(transferBytes.end(), bytes.begin(), bytes.end());
    setChipSelect(false);
    ftdiWrite(transferBytes);
    setChipSelect(true);
    return read(static_cast<DWORD>(bytes.size()));
}

void Spi::ftdiWrite(const Spi::Bytes& bytes) {
    Bytes buffer = bytes;
    DWORD bytesWritten = 0;
    FT_STATUS ftStatus = FT_Write(ftHandle_, buffer.data(),
            static_cast<DWORD>(buffer.size()), &bytesWritten);
    if (ftStatus != FT_OK) {
        throw Spi::WriteException(ftStatus);
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
            throw MpsseFailedException();
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


Spi::OpenPortException::OpenPortException() :
        runtime_error("Failed to open the SPI port.\n") {}

Spi::OpenPortException::OpenPortException(int port) :
        runtime_error("Port " + std::to_string(port) +
                " is already in use.\n") {}

Spi::OpenPortException::OpenPortException(int port, FT_STATUS status) :
        runtime_error("FT_Open (port " + std::to_string(port) +
                ") failed with FT_STATUS = " + std::to_string(status) +
                ".\n") {}


Spi::ReadException::ReadException() :
        runtime_error("Failed to read data from SPI.\n") {}

Spi::ReadException::ReadException(FT_STATUS status) :
        runtime_error("FT_Read failed with FT_STATUS = " +
                std::to_string(status) + ".\n") {}

Spi::ReadException::ReadException(unsigned attempts, FT_STATUS status) :
        runtime_error("Too much attempts of FT_GetQueueStatus (" +
                std::to_string(attempts) + "). " +
                "The most recent FT_STATUS = " + std::to_string(status) +
                ".\n") {}


Spi::WriteException::WriteException() :
        runtime_error("Failed to write data to SPI.\n") {}

Spi::WriteException::WriteException(FT_STATUS status) :
        runtime_error("FT_Write failed with FT_STATUS = " +
                std::to_string(status) + ".\n") {}


Spi::MpsseFailedException::MpsseFailedException() :
        runtime_error("MPSSE did not respond correctly.\n") {}

Spi::MpsseFailedException::MpsseFailedException(const std::string& description) :
        runtime_error("MPSSE failed: " + description + ".\n") {}


} // namespace d2xxwrapper

} // namespace controller
