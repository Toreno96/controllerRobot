#ifndef _WRAPPERS_DYNAMIXEL3_SYNC_READER_H_
#define _WRAPPERS_DYNAMIXEL3_SYNC_READER_H_

#include <vector>
#include "../../../3rdParty/dynamixel3/include/dynamixel_sdk.h"
#include "Wrappers/dynamixel3/globals.h"

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
class SyncReader {
    public:
        SyncReader( dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    tAddress data );
        template< typename U, typename UnaryFunction >
        std::vector< U > read( const std::vector< tId >& receivers,
                               const UnaryFunction& converter );
        std::vector< T > read( const std::vector< tId >& receivers );
    private:
        dynamixel::GroupSyncRead internalReader_;
        tAddress data_;
};

} // namespace dynamixel3wrapper

} // namespace controller

#include "Wrappers/dynamixel3/syncReader.tpp"

#endif // #ifndef _WRAPPERS_DYNAMIXEL3_SYNC_READER_H_