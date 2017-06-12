///
/// \file
/// \brief Wrapper of Dynamixel SDK's GroupSyncWrite (experimental).
/// \author Daniel Staśczak
///

#include "syncWriter.h"

template< typename T >
SyncWriter< T >::SyncWriter( dummy::PortHandler* portHandler,
                             dummy::PacketHandler* packetHandler,
                             tAddress data ) :
    internalWriter_( portHandler, packetHandler, data, getDataLength() ),
    portHandler_( portHandler ), packetHandler_( packetHandler ),
    data_( data ) {}
template<>
uint16_t SyncWriter< uint8_t >::getDataLength() {
  return 1;
}
template<>
uint16_t SyncWriter< uint16_t >::getDataLength() {
  return 2;
}
template<>
uint16_t SyncWriter< uint32_t >::getDataLength() {
  return 4;
}
template< typename T >
void SyncWriter< T >::send( const std::vector< tId >& receivers,
                       const std::vector< T >& values ) {
  auto itValues = values.begin();
  for( auto receiver : receivers ) {
    uint8_t convertedValue = convert( *itValues++ );
    internalWriter_.addParam( receiver, &convertedValue );
  }

  // communicationResult = internalWriter_.txPacket();
  // handle( communicationResult );
}
template<>
uint8_t SyncWriter< uint8_t >::convert( uint8_t value ) {
  return 8;
}
template<>
uint8_t SyncWriter< uint16_t >::convert( uint16_t value ) {
  return 16;
}
template<>
uint8_t SyncWriter< uint32_t >::convert( uint32_t value ) {
  return 32;
}