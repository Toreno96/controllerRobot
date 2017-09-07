///
/// \file
/// \brief Helper for group communication.
/// \author Daniel Staśczak
///

#include <type_traits>
#include "Board/Galgo/groupCommunicationHelper.h"

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
constexpr void GroupCommunicationHelper< T >::checkType() {
    static_assert( std::is_same< T, uint8_t >::value ||
            std::is_same< T, uint16_t >::value ||
            std::is_same< T, uint32_t >::value,
            "Such type of the T is not supported" );
}
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