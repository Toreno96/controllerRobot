///
/// \file
/// \brief Dummy version of the most essential Dynamixel SDK's classes.
/// \author Daniel Staśczak
///

#include "dummies.h"

#include <iostream>

namespace dummy {

GroupSyncWrite::GroupSyncWrite( PortHandler* portHandler,
                                PacketHandler* packetHandler,
                                uint16_t address, uint16_t dataLength ) :
    portHandler_( portHandler ), packetHandler_( packetHandler ),
    address_( address ), dataLength_( dataLength ) {}
void GroupSyncWrite::addParam( uint8_t id, uint8_t* value ) {
  std::cout << "Value: " << std::to_string( *value ) << '\n'
      << "ID: " << std::to_string( id ) << '\n';
}

} // namespace dummy