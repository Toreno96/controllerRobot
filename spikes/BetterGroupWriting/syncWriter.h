///
/// \file
/// \brief Wrapper of Dynamixel SDK's GroupSyncWrite (experimental).
/// \author Daniel Staśczak
///

#pragma once

#include <cstdint>
#include <vector>
#include "dummies.h"

using tAddress = uint16_t;
using tId = uint8_t;

template< typename T /*std::enable_if...*/ >
class SyncWriter {
  public:
    SyncWriter( dummy::PortHandler* portHandler,
                dummy::PacketHandler* packetHandler,
                tAddress data );
    void send( const std::vector< tId >& receivers,
               const std::vector< T >& values );
  private:
    uint16_t getDataLength();
    uint8_t convert( T value );
    dummy::GroupSyncWrite internalWriter_;
    dummy::PortHandler* portHandler_;
    dummy::PacketHandler* packetHandler_;
    tAddress data_;
};

#ifdef PSEUDOCODE
SyncWriter writer< uint8_t >( portHandlersByLegNumber_.at( legNo ).get(),
    packetHandler_.get(), LED );
auto jointsIds = getSingleLegIds( legNo );
// Convert powered to proper data type
writer.send( jointsIds, powered );
#endif // #ifdef PSEUDOCODE

#include "syncWriter.tpp"