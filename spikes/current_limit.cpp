#include <cassert>
#include <memory>
#include <iostream>
#include "dynamixel_sdk.h"

int main() {
  using tPortHandler = std::shared_ptr< dynamixel::PortHandler >;
  using tPacketHandler = std::shared_ptr< dynamixel::PacketHandler >;
  
  tPortHandler portHandler( dynamixel::PortHandler::getPortHandler( "/dev/ttyUSB0" ) );
  if( !portHandler->openPort() )
    std::cout << "Cannot open port\n";
  if( !portHandler->setBaudRate( 3000000 ) )
    std::cout << "Cannot set baudrate\n";
  tPacketHandler packetHandler( dynamixel::PacketHandler::getPacketHandler( 2.0 ) );

  uint8_t error;
  int communicationResult = packetHandler->write1ByteTxRx( portHandler.get(),
      11, 64, 1, &error );
  if( error != 0 )
    packetHandler->printRxPacketError( error );
  if( communicationResult != COMM_SUCCESS )
    packetHandler->printTxRxResult( communicationResult );

  uint16_t data;
  std::cin >> data;
  communicationResult = packetHandler->write2ByteTxRx( portHandler.get(), 11,
      102, data, &error);
  if( error != 0 )
    packetHandler->printRxPacketError( error );
  if( communicationResult != COMM_SUCCESS )
    packetHandler->printTxRxResult( communicationResult );
}