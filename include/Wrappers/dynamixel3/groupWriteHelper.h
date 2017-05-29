#ifndef _WRAPPERS_DYNAMIXEL3_GROUP_WRITE_HELPER_H_
#define _WRAPPERS_DYNAMIXEL3_GROUP_WRITE_HELPER_H_

#include <array>
#include "groupCommunicationHelper.h"

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
class SyncWriter;

template< typename T >
class GroupWriteHelper {
    public:
      GroupWriteHelper() = delete;
    private:
        using tLittleEndian = std::array<
                uint8_t, GroupCommunicationHelper< T >::getDataLength() >;
        static tLittleEndian toLittleEndian( T value );
        friend class SyncWriter< T >;
};

} // namespace dynamixel3wrapper

} // namespace controller

#include "groupWriteHelper.tpp"

#endif // #ifndef _WRAPPERS_DYNAMIXEL3_GROUP_WRITE_HELPER_H_