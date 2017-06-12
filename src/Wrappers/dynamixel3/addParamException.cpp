///
/// \file
/// \brief Add param exception.
/// \author Daniel Staśczak
///

#include "Wrappers/dynamixel3/addParamException.h"

namespace controller {

namespace dynamixel3wrapper {

AddParamException::AddParamException() :
        std::runtime_error( "Add param unsuccessful" ) {}

} // namespace dynamixel3wrapper

} // namespace controller