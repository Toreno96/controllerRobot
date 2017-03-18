#include "Board/boardGalgo.h"
#include "Board/exceptions.h"

namespace controller {

// TO-DO Zapytać o Type
BoardGalgo::BoardGalgo( const std::string &port, int baudRate ) :
    Board( "Board Galgo", TYPE_USB2DYNAMIXEL ) {
  if( !portHandler->openPort() )
      throw FailedOpeningPortException("Failed to open the port \"" + port + '\"');
  if( !portHandler->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException("Failed to change the baudrate to " + baudRate);
}
BoardGalgo::~BoardGalgo() {
  portHandler->closePort();
}

} // namespace controller
