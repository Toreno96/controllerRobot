///
/// \file
/// \brief Helper for group communication.
/// \author Daniel Staśczak
///

#ifndef _WRAPPERS_DYNAMIXEL3_GROUP_COMMUNICATION_HELPER_H_
#define _WRAPPERS_DYNAMIXEL3_GROUP_COMMUNICATION_HELPER_H_

namespace controller {

namespace dynamixel3wrapper {

template< typename T >
class GroupWriteHelper;
template< typename T >
class SyncWriter;
template< typename T >
class SyncReader;

template< typename T >
class GroupCommunicationHelper {
    public:
        GroupCommunicationHelper() = delete;
    private:
        static constexpr void checkType();
        static constexpr uint16_t getDataLength();
        friend class GroupWriteHelper< T >;
        friend class SyncWriter< T >;
        friend class SyncReader< T >;
};

} // namespace dynamixel3wrapper

} // namespace controller

#include "Board/Galgo/groupCommunicationHelper.tpp"

#endif // #ifndef _WRAPPERS_DYNAMIXEL3_GROUP_COMMUNICATION_HELPER_H_