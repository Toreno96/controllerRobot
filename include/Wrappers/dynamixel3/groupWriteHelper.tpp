#include "groupWriteHelper.h"

namespace controller {

namespace dynamixel3wrapper {

template<>
GroupWriteHelper< uint8_t >::tLittleEndian
        GroupWriteHelper< uint8_t >::toLittleEndian( uint8_t value ) {
    tLittleEndian valueInLittleEndian;
    valueInLittleEndian.at( 0 ) = value;
    return valueInLittleEndian;
}
template<>
GroupWriteHelper< uint16_t >::tLittleEndian
        GroupWriteHelper< uint16_t >::toLittleEndian( uint16_t value ) {
    tLittleEndian valueInLittleEndian;
    valueInLittleEndian.at( 0 ) = DXL_LOBYTE( value );
    valueInLittleEndian.at( 1 ) = DXL_HIBYTE( value );
    return valueInLittleEndian;
}
template<>
GroupWriteHelper< uint32_t >::tLittleEndian
        GroupWriteHelper< uint32_t >::toLittleEndian( uint32_t value ) {
    tLittleEndian valueInLittleEndian;
    valueInLittleEndian.at( 0 ) = DXL_LOBYTE( DXL_LOWORD( value ) );
    valueInLittleEndian.at( 1 ) = DXL_HIBYTE( DXL_LOWORD( value ) );
    valueInLittleEndian.at( 2 ) = DXL_LOBYTE( DXL_HIWORD( value ) );
    valueInLittleEndian.at( 3 ) = DXL_HIBYTE( DXL_HIWORD( value ) );
    return valueInLittleEndian;
}

} // namespace dynamixel3wrapper

} // namespace controller