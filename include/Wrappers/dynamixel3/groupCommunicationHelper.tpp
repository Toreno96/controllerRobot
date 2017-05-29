#include "Wrappers/dynamixel3/groupCommunicationHelper.h"

namespace controller {

namespace dynamixel3wrapper {

template<>
constexpr uint16_t GroupCommunicationHelper< uint8_t >::getDataLength() {
    return 1;
}
template<>
constexpr uint16_t GroupCommunicationHelper< uint16_t >::getDataLength() {
    return 2;
}
template<>
constexpr uint16_t GroupCommunicationHelper< uint32_t >::getDataLength() {
    return 4;
}

} // namespace dynamixel3wrapper

} // namespace controller