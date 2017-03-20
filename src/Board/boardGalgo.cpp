#include "../../3rdParty/dynamixel3/include/dynamixel_sdk.h"
#include "Board/boardGalgo.h"
#include "Board/exceptions.h"

namespace controller {

// TO-DO Zapytać o Type
BoardGalgo::BoardGalgo( const std::string &port, int baudRate ) :
    Board( "Board Galgo", TYPE_USB2DYNAMIXEL ), protocolVersion_( 2.0 ),
    portHandler_( dynamixel::PortHandler::getPortHandler( port.c_str() ) ) {
  if( !portHandler_->openPort() )
      throw FailedOpeningPortException("Failed to open the port \"" + port + '\"');
  if( !portHandler_->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException("Failed to change the baudrate to " + baudRate);
}
BoardGalgo::~BoardGalgo() {
  portHandler_->closePort();
}

void BoardGalgo::toggleTorque( uint8_t dynamixelId, bool onOrOff ) {
    uint8_t error;
    dynamixel::PacketHandler *packetHandler =
            dynamixel::PacketHandler::getPacketHandler( protocolVersion_ );
    int communicationResult = packetHandler->write1ByteTxRx( portHandler_,
            dynamixelId, 64, onOrOff, &error );

    if( communicationResult != COMM_SUCCESS ) {
        packetHandler->printTxRxResult( communicationResult );
    }
    else if( error != 0 ) {
        packetHandler->printRxPacketError( error );
    }
}

} // namespace controller
