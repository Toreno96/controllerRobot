#ifndef _WRAPPERS_DYNAMIXEL3_COMMUNICATION_RESULT_H_
#define _WRAPPERS_DYNAMIXEL3_COMMUNICATION_RESULT_H_

#include <cstdint>
#include "../../../3rdParty/dynamixel3/include/dynamixel_sdk.h"

namespace controller {

namespace dynamixel3wrapper {

class CommunicationResult {
    public:
        CommunicationResult( dynamixel::PacketHandler* packetHandler,
                             int result, uint8_t error = 0 );
        void handle();
    private:
        void handleResult();
        void handleError();
        dynamixel::PacketHandler* const packetHandler_;
        int result_;
        uint8_t error_;
};

} // namespace dynamixel3wrapper

} // namespace controller

#endif // #ifndef _WRAPPERS_DYNAMIXEL3_COMMUNICATION_RESULT_H_