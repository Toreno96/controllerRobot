///
/// \file
/// \brief Wrapper of Dynamixel SDK's GroupSyncWrite.
/// \author Daniel Staśczak
///

#ifndef _WRAPPERS_DYNAMIXEL3_SYNC_WRITER_H_
#define _WRAPPERS_DYNAMIXEL3_SYNC_WRITER_H_

#include <vector>
#include "3rdParty/dynamixel3/include/dynamixel_sdk.h"
#include "Board/Galgo/globals.h"

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
class SyncWriter {
    public:
        SyncWriter( dynamixel::PortHandler* portHandler,
                    dynamixel::PacketHandler* packetHandler,
                    tAddress data );
        template< typename Iterator, typename UnaryFunction >
        Iterator write( const std::vector< tId >& receivers,
                        Iterator values,
                        const UnaryFunction& converter );
        template< typename U, typename UnaryFunction >
        void write( const std::vector< tId >& receivers,
                        const std::vector< U >& values,
                        const UnaryFunction& converter );
        template< typename Iterator >
        Iterator write( const std::vector< tId >& receivers,
                        Iterator values );
        void write( const std::vector< tId >& receivers,
                        const std::vector< T >& values );
    private:
        dynamixel::GroupSyncWrite internalWriter_;
        tAddress data_;
};

} // namespace dynamixel3wrapper

} // namespace controller

#include "Board/Galgo/syncWriter.tpp"

#endif // #ifndef _WRAPPERS_DYNAMIXEL3_SYNC_WRITER_H_