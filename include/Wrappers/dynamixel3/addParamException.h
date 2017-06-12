#ifndef _WRAPPERS_DYNAMIXEL3_ADD_PARAM_EXCEPTION_H_
#define _WRAPPERS_DYNAMIXEL3_ADD_PARAM_EXCEPTION_H_

#include <stdexcept>

namespace controller {

namespace dynamixel3wrapper {

class AddParamException : public std::runtime_error {
    public:
        AddParamException();
};

} // namespace dynamixel3wrapper

} // namespace controller

#endif // #ifndef _WRAPPERS_DYNAMIXEL3_ADD_PARAM_EXCEPTION_H_