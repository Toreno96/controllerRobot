///
/// \file
/// \brief Exceptions for BoardGalgo class.
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

#include "Board/Galgo/exceptions.h"

namespace controller {

FailedOpeningPortException::FailedOpeningPortException():
    runtime_error("Failed to open the USB port.\n"){}

FailedOpeningPortException::FailedOpeningPortException(const string& portName):
    runtime_error("Failed to open the port " + portName + ".\n"){}


FailedChangingBaudRateException::FailedChangingBaudRateException():
    runtime_error("Failed to change the baudrate.\n"){}

FailedChangingBaudRateException::FailedChangingBaudRateException(int baudRate):
    runtime_error("Failed to change the baudrate to " + to_string(baudRate) + ".\n"){}


NotSupportedException::NotSupportedException():
    runtime_error("This function is not supported.\n"){}

NotSupportedException::NotSupportedException(const string& description):
    runtime_error("This function is not supported: " + description + "\n"){}


FailedLoadingGalgoConfigException::FailedLoadingGalgoConfigException(
                const std::string& filename) :
        std::runtime_error("Unable to load board Galgo config file '" +
                filename + "'") {}

InvalidDataFromSpiException::InvalidDataFromSpiException() :
        std::runtime_error("Data read from SPI is invalid") {}


} // namespace controller
