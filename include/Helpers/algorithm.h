#ifndef _HELPERS_ALGORITHM_H_
#define _HELPERS_ALGORITHM_H_

#include <algorithm>
#include <array>

namespace controller {

template< typename T, std::size_t In1, std::size_t In2 >
std::array< T, In1 + In2 > merge( const std::array< T, In1 >& in1,
                                  const std::array< T, In2 >& in2 );

} // namespace controller

#include "Helpers/algorithm.tpp"

#endif // #ifndef _HELPERS_ALGORITHM_H_