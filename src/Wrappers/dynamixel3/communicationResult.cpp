#include <iostream>
#include "Wrappers/dynamixel3/communicationResult.h"

namespace controller {

namespace dynamixel3wrapper {

CommunicationResult::CommunicationResult( dynamixel::PacketHandler*
                                                  packetHandler,
                                          int result, uint8_t error ) :
        packetHandler_( packetHandler ), result_( result ), error_( error ) {}
// TO-DO Customowe wyjątki;
// umożliwienie wypisania TxRxResult _po_ złapaniu wyjątku
void CommunicationResult::handle() {
    #ifdef DEBUG
    handleResult();
    #endif // #ifdef DEBUG
    handleError();
}
void CommunicationResult::handleResult() {
    if( result_ != COMM_SUCCESS ) {
        std::cout << '\t';
        packetHandler_->printTxRxResult( result_ );
        throw std::runtime_error( "Dynamixel communication unsuccessful" );
    }
}
void CommunicationResult::handleError() {
    if( error_ != 0 ) {
        std::cout << "Dynamixel error:\n\t";
        packetHandler_->printRxPacketError( error_ );
    }
}

} // namespace dynamixel3wrapper

} // namespace controller