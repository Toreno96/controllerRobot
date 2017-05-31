#include "Board/exceptions.h"

controller::FailedOpeningPortException::FailedOpeningPortException():
    runtime_error("Failed to open the USB port.\n"){}

controller::FailedOpeningPortException::FailedOpeningPortException(const string& portName):
    runtime_error("Failed to open the port " + portName + ".\n"){}


controller::FailedChangingBaudRateException::FailedChangingBaudRateException():
    runtime_error("Failed to change the baudrate.\n"){}

controller::FailedChangingBaudRateException::FailedChangingBaudRateException(int baudRate):
    runtime_error("Failed to change the baudrate to " + to_string(baudRate) + ".\n"){}


controller::NotSupportedException::NotSupportedException():
    runtime_error("This function is not supported.\n"){}

controller::NotSupportedException::NotSupportedException(const string& description):
    runtime_error("This function is not supported: " + description + "\n"){}
