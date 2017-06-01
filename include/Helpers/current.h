#ifndef _HELPERS_CURRENT_H_
#define _HELPERS_CURRENT_H_

#include <cstdint>
#include <type_traits>

namespace controller {

template< typename T, int Dynamixel, int Interval, int Ampers >
struct CurrentUnit {
    using value_type = T;
};

using tCurrentUnitDynamixel = CurrentUnit< uint16_t, 1, 0, 0 >;
using tCurrentUnitInterval = CurrentUnit< double, 0, 1, 0 >;
using tCurrentUnitAmpers = CurrentUnit< double, 0, 0, 1 >;

template< typename U >
class Current {
    private:
        typename U::value_type val_;
    public:
        explicit Current( typename U::value_type d ) : val_( d ) {}
        typename U::value_type val() {
            return val_;
        }

        template< typename T = U >
        operator Current< typename std::enable_if_t<
                !std::is_same< T, tCurrentUnitDynamixel >::value,
                tCurrentUnitDynamixel > >();
        template< typename T = U >
        operator Current< typename std::enable_if_t<
                !std::is_same< T, tCurrentUnitInterval >::value,
                tCurrentUnitInterval > >();
        template< typename T = U >
        operator Current< typename std::enable_if_t<
                !std::is_same< T, tCurrentUnitAmpers >::value,
                tCurrentUnitAmpers > >();

        static constexpr typename U::value_type full();
};

} // namespace controller

#include "Helpers/current.tpp"

#endif // #ifndef _HELPERS_CURRENT_H_