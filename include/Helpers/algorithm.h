///
/// \file
/// \brief General-purpose helper algorithms.
/// \author Daniel Staśczak
///

#ifndef _HELPERS_ALGORITHM_H_
#define _HELPERS_ALGORITHM_H_

#include <algorithm>
#include <vector>

namespace controller {

/// Merges two sorted vectors into one sorted vector
template< typename T >
std::vector< T > merge( const std::vector< T >& in1,
                        const std::vector< T >& in2 );
/// Merges two vectors into one vector (neither in1 nor in2 is required to be sorted)
template< typename T >
std::vector< T > unsortedMerge( const std::vector< T >& in1,
                                const std::vector< T >& in2 );

} // namespace controller

#include "Helpers/algorithm.tpp"

#endif // #ifndef _HELPERS_ALGORITHM_H_