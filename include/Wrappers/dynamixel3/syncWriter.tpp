#include "Wrappers/dynamixel3/communicationResult.h"
#include "Wrappers/dynamixel3/groupWriteHelper.h"
#include "Wrappers/dynamixel3/syncWriter.h"

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
SyncWriter< T >::SyncWriter( dynamixel::PortHandler* portHandler,
                             dynamixel::PacketHandler* packetHandler,
                             tAddress data ) :
        internalWriter_( portHandler, packetHandler, data,
                GroupCommunicationHelper< T >::getDataLength() ),
        data_( data ) {
    GroupCommunicationHelper< T >::checkType();
}
template< typename T > template< typename Iterator, typename UnaryFunction >
Iterator SyncWriter< T >::write( const std::vector< tId >& receivers,
                                 Iterator values,
                                 const UnaryFunction& converter ) {
  internalWriter_.clearParam();
  for( auto receiver : receivers ) {
        T valueInDynamixel = converter( *values++ );
        auto valueInLittleEndian =
                GroupWriteHelper< T >::toLittleEndian( valueInDynamixel );
        if( !internalWriter_.addParam( receiver, valueInLittleEndian.data() ) )
            // TO-DO Customowy wyjątek o bardziej przemyślanej treści
            throw std::runtime_error( "Add param unsuccessful" );
  }
  CommunicationResult communicationResult( internalWriter_.getPacketHandler(),
        internalWriter_.txPacket() );
  communicationResult.handle();
  return values;
}
template< typename T > template< typename U, typename UnaryFunction >
void SyncWriter< T >::write( const std::vector< tId >& receivers,
                             const std::vector< U >& values,
                             const UnaryFunction& converter ) {
    write( receivers, values.begin(), converter );
}
template< typename T > template< typename Iterator >
Iterator SyncWriter< T >::write( const std::vector< tId >& receivers,
                                 Iterator values ) {
    return write( receivers, values, []( T value ) {
        return value;
    } );
}
template< typename T >
void SyncWriter< T >::write( const std::vector< tId >& receivers,
                             const std::vector< T >& values ) {
    write( receivers, values.begin() );
}

} // namespace dynamixel3wrapper

} // namespace controller