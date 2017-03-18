#include "Board/exceptions.h"

controller::FailedOpeningPortException::FailedOpeningPortException(string description){
    this->_desc = description;
}

const char* controller::FailedOpeningPortException::what() const throw(){
    return this->_desc.c_str();
}


controller::FailedChangingBaudRateException::FailedChangingBaudRateException(string description){
    this->_desc = description;
}

const char* controller::FailedChangingBaudRateException::what() const throw(){
    return this->_desc.c_str();
}
