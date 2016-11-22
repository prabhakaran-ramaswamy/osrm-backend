#ifndef HILBERT_VALUE_HPP
#define HILBERT_VALUE_HPP

#include "osrm/coordinate.hpp"

#include <climits>
#include <cstdint>

namespace osrm
{
namespace util
{
namespace
{
// Arndt, Jörg. Matters Computational Ideas, Algorithms, Source Code, 2010.
// Figure 1.31-D: the finite state machine for the 2-dimensional Hilbert inverse function (left)
// http://home.pipeline.com/~hbaker1/hakmem/topology.html#item115
static const std::uint8_t ihtab[] = {
#define IHT(ai,bi,c0,c1) ((ai<<3)+(bi<<2)+(c0<<1)+(c1))
    // index == HT(c0,c1,xi,yi)
    IHT(0, 0, 1, 0),
    IHT(0, 1, 0, 0),
    IHT(1, 1, 0, 1),
    IHT(1, 0, 0, 0),
    IHT(1, 0, 0, 1),
    IHT(0, 1, 0, 1),
    IHT(1, 1, 0, 0),
    IHT(0, 0, 1, 1),
    IHT(0, 0, 0, 0),
    IHT(1, 1, 1, 1),
    IHT(0, 1, 1, 0),
    IHT(1, 0, 1, 0),
    IHT(1, 0, 1, 1),
    IHT(1, 1, 1, 0),
    IHT(0, 1, 1, 1),
    IHT(0, 0, 0, 1)
#undef IHT
};
}

// Transform Hilbert x and y to linear coordinate t
// Arndt, Jörg. Matters Computational Ideas, Algorithms, Source Code, 2010.
// 1.31.1 The Hilbert curve p. 86
template<int N = 32, typename T = std::uint32_t, typename R = std::uint64_t>
inline R HilbertToLinear(T x, T y)
{
    static_assert(N <= sizeof(T) * CHAR_BIT, "input type is smaller than N");
    static_assert(2 * N <= sizeof(R) * CHAR_BIT, "output type is smaller than 2N");

    R t = 0;
    std::uint8_t c01 = 0;
    for (int i = 0; i < N; ++i)
    {
        t <<= 2;
        const T xi = (x >> (sizeof(T) * CHAR_BIT - 1)) & 1;
        const T yi = (y >> (sizeof(T) * CHAR_BIT - 1)) & 1;
        x <<= 1;
        y <<= 1;
        const std::uint8_t st = ihtab[(c01<<2) | (xi<<1) | yi];
        c01 = st & 3;
        t |= (st>>2);
    }

    return t;
}

// Computes a 64 bit value that corresponds to the hilbert space filling curve
inline std::uint64_t GetHilbertCode(const Coordinate &coordinate)
{
    const std::uint32_t x = static_cast<std::int32_t>(coordinate.lon) +
        static_cast<std::int32_t>(180 * COORDINATE_PRECISION);
    const std::uint32_t y = static_cast<std::int32_t>(coordinate.lat) +
        static_cast<std::int32_t>(90 * COORDINATE_PRECISION);
    return HilbertToLinear(x, y);
}

}
}

#endif /* HILBERT_VALUE_HPP */
