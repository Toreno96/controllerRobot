#include "Board/boardGalgo.h"
#include "Board/exceptions.h"

namespace controller {

// TO-DO Zapytać o Type
BoardGalgo::BoardGalgo( const std::string &port, int baudRate ) :
    Board( "Board Galgo", TYPE_USB2DYNAMIXEL ) {
    portHandler = dynamixel::PortHandler::getPortHandler(port);
  if( !portHandler->openPort() )
      throw FailedOpeningPortException("Failed to open the port \"" + port + '\"');
  if( !portHandler->setBaudRate( baudRate ) )
      throw FailedChangingBaudRateException("Failed to change the baudrate to " + baudRate);

  packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);
}
BoardGalgo::~BoardGalgo() {
  portHandler->closePort();
}

BoardGalgo::setLED(int legNo, int jointNo, bool powered){
    uint8_t id = legNo * 10 + jointNo;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_LED, powered, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS){
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0) {
      packetHandler->printRxPacketError(dxl_error);
    }
}

} // namespace controller
