#include "Helpers/algorithm.h"

namespace controller {

template< typename T, std::size_t In1, std::size_t In2 >
std::array< T, In1 + In2 > merge( const std::array< T, In1 >& in1,
                                  const std::array< T, In2 >& in2 ) {
    std::array< T, In1 + In2 > out;
    std::merge( in1.begin(), in1.end(), in2.begin(), in2.end(), out.begin() );
    return out;
}

} // namespace controller