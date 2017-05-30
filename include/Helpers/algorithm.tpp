#include "Helpers/algorithm.h"

namespace controller {

template< typename T >
std::vector< T > merge( const std::vector< T > in1,
                        const std::vector< T > in2 ) {
    std::vector< T > out( in1.size() + in2.size() );
    std::merge( in1.begin(), in1.end(), in2.begin(), in2.end(), out.begin() );
    return out;
}

} // namespace controller