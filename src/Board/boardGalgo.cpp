#include "Board/boardGalgo.h"

namespace controller {

// TO-DO Zapytać o Type
BoardGalgo::BoardGalgo( const std::string &port, int baudRate ) :
    Board( "Board Galgo", TYPE_USB2DYNAMIXEL ) {
  // TO-DO Zamienić na customowe wyjątki
  if( !portHandler->openPort() )
    throw std::runtime_error( "Failed to open the port \"" + port + '\"' );
  if( !portHandler->setBaudRate( baudRate ) )
    throw std::runtime_error( "Failed to change the baudrate to " + baudRate );
}
BoardGalgo::~BoardGalgo() {
  portHandler->closePort();
}

} // namespace controller