///
/// \file
/// \brief Wrapper of Dynamixel SDK's GroupSyncRead (test).
/// \author Daniel Staśczak
///

// Compile with: clang++ -std=c++14  ./tests/syncReader.cpp ./src/Wrappers/dynamixel3/communicationResult.cpp ./3rdParty/dynamixel3/src/dynamixel_sdk/*.cpp ./3rdParty/dynamixel3/src/dynamixel_sdk_linux/port_handler_linux.cpp -I./include/ -I./3rdParty/dynamixel3/include/ -o ./tests/syncReader.out

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include "Wrappers/dynamixel3/syncReader.h"

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
    SyncReader< uint8_t > reader( portHandler.get(), packetHandler.get(), 65 );
    // Compilation error if uncommented
    // SyncReader< bool > invalidReader( portHandler.get(), packetHandler.get(), 65 );
    
    std::vector< tId > receivers{ 11, 12, 13, 21, 22, 23 };
    auto values = reader.read( receivers );
    for( auto value : values )
        std::cout << value << ' ';
    std::cout << '\n';
}