#ifndef _HELPERS_ALGORITHM_H_
#define _HELPERS_ALGORITHM_H_

#include <algorithm>
#include <vector>

namespace controller {

template< typename T >
std::vector< T > merge( const std::vector< T >& in1,
                        const std::vector< T >& in2 );

} // namespace controller

#include "Helpers/algorithm.tpp"

#endif // #ifndef _HELPERS_ALGORITHM_H_