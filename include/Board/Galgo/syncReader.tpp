///
/// \file
/// \brief Wrapper of Dynamixel SDK's GroupSyncRead.
/// \author Daniel Staśczak
///

#include "Board/Galgo/addParamException.h"
#include "Board/Galgo/communicationResult.h"
#include "Board/Galgo/groupCommunicationHelper.h"
#include "Board/Galgo/syncReader.h"

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
SyncReader< T >::SyncReader( dynamixel::PortHandler* portHandler,
                             dynamixel::PacketHandler* packetHandler,
                             tAddress data ) :
        internalReader_( portHandler, packetHandler, data,
                GroupCommunicationHelper< T >::getDataLength() ),
        data_( data ) {
    GroupCommunicationHelper< T >::checkType();
}
template< typename T > template< typename U, typename UnaryFunction >
std::vector< U > SyncReader< T >::read( const std::vector< tId >& receivers,
                                        const UnaryFunction& converter ) {
    internalReader_.clearParam();
    for( auto receiver : receivers ) {
        if( !internalReader_.addParam( receiver ) )
            throw AddParamException();
    }
    CommunicationResult communicationResult( internalReader_.getPacketHandler(),
            internalReader_.txRxPacket() );
    communicationResult.handle();
    std::vector< U > values( receivers.size() );
    auto receiver = receivers.begin();
    for( auto& value : values ) {
        value = converter( static_cast< T >( internalReader_.getData( *receiver++, data_,
                GroupCommunicationHelper< T >::getDataLength() ) ) );
    }
    return values;
}
template< typename T >
std::vector< T > SyncReader< T >::read( const std::vector< tId >& receivers ) {
    return read< T >( receivers, []( T value ){
        return value;
    } );
}

} // namespace dynamixel3wrapper

} // namespace controller