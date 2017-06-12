///
/// \file
/// \brief Wrapper of Dynamixel SDK's GroupSyncWrite (test).
/// \author Daniel Staśczak
///

// Compile with: clang++ -std=c++14  ./tests/syncWriter.cpp ./src/Wrappers/dynamixel3/communicationResult.cpp ./3rdParty/dynamixel3/src/dynamixel_sdk/*.cpp ./3rdParty/dynamixel3/src/dynamixel_sdk_linux/port_handler_linux.cpp -I./include/ -I./3rdParty/dynamixel3/include/ -o ./tests/syncWriter.out

#include <chrono>
#include <memory>
#include <thread>
#include <vector>
#include "Wrappers/dynamixel3/syncWriter.h"

int main() {
    using namespace controller::dynamixel3wrapper;
    using tPortHandler = std::shared_ptr< dynamixel::PortHandler >;
    using tPacketHandler = std::shared_ptr< dynamixel::PacketHandler >;
    
    tPortHandler portHandler( dynamixel::PortHandler::getPortHandler(
            "/dev/ttyUSB0" ) );
    portHandler->openPort();
    portHandler->setBaudRate( 3000000 );
    tPacketHandler packetHandler(
            dynamixel::PacketHandler::getPacketHandler( 2.0 ) );
    SyncWriter< uint8_t > writer( portHandler.get(), packetHandler.get(), 65 );
    // Compilation error if uncommented
    // SyncWriter< bool > invalidWriter( portHandler.get(), packetHandler.get(), 65 );

    std::vector< tId > receivers1{ 11, 12, 13, 21, 22, 23 };
    std::vector< uint8_t > on( 6, 1 );
    std::vector< uint8_t > off( 6, 0 );
    for( int i = 0; i < 3; ++i ) {
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        writer.write( receivers1, on );
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        writer.write( receivers1, off );
    }

    std::vector< tId > receivers2{ 11, 12, 13 };
    std::vector< uint8_t > values{ 1, 0, 1, 0, 1, 0 };
    for( int i = 0; i < 3; ++i ) {
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        auto it = writer.write( receivers2, values.begin() );
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        writer.write( receivers2, it );
    }
}