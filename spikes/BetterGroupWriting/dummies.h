#pragma once

#include <cstdint>

namespace dummy {

class PortHandler {};

class PacketHandler {};

class GroupSyncWrite {
  public:
    GroupSyncWrite( PortHandler* portHandler,
                    PacketHandler* packetHandler,
                    uint16_t address, uint16_t dataLength );
    void addParam( uint8_t id, uint8_t* value );
  private:
    PortHandler* portHandler_;
    PacketHandler* packetHandler_;
    uint16_t address_;
    uint16_t dataLength_;
};

} // namespace dummy