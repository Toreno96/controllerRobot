///
/// \file
/// \brief Exceptions for BoardGalgo class.
/// \author Daniel Staśczak
/// \author Marcin Orczyk
///

#ifndef exceptions_h
#define exceptions_h

#include <stdexcept>
#include <string>

using std::string;
using std::to_string;
using std::runtime_error;


namespace controller{


class FailedOpeningPortException: public runtime_error{
public:
    FailedOpeningPortException();
    FailedOpeningPortException(const string& portName);
};

class FailedChangingBaudRateException: public runtime_error{
public:
    FailedChangingBaudRateException();
    FailedChangingBaudRateException(int baudRate);
};

class NotSupportedException: public runtime_error{
public:
    NotSupportedException();
    NotSupportedException(const string& description);
};

}

#endif
