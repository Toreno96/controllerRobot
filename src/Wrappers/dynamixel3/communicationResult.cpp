#include <iostream>
#include "Wrappers/dynamixel3/communicationResult.h"

namespace controller {

namespace dynamixel3wrapper {

CommunicationResult::Unsuccessful::Unsuccessful( const std::string& result ) :
        std::runtime_error( "Dynamixel communication unsuccessful:\n\t" +
                result ) {}
CommunicationResult::CommunicationResult( dynamixel::PacketHandler*
                                                  packetHandler,
                                          int result, uint8_t error ) :
        packetHandler_( packetHandler ), result_( result ), error_( error ) {}
void CommunicationResult::handle() {
    #ifdef DEBUG
    handleResult();
    #endif // #ifdef DEBUG
    handleError();
}
void CommunicationResult::handleResult() {
    if( result_ != COMM_SUCCESS )
        throw Unsuccessful( packetHandler_->getTxRxResult( result_ ) );
}
void CommunicationResult::handleError() {
    if( error_ != 0 ) {
        std::cout << "Dynamixel error:\n\t";
        packetHandler_->printRxPacketError( error_ );
    }
}

} // namespace dynamixel3wrapper

} // namespace controller