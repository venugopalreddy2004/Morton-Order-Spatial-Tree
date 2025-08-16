

#pragma once

#include <array>
#include <cstdint>
#include <boost/geometry.hpp>

const size_t D = 6;

struct Point6D : public std::array<uint32_t, D>
{
};

namespace boost
{
    namespace geometry
    {
        namespace traits
        {

            template <>
            struct tag<Point6D>
            {
                using type = point_tag;
            };

            template <>
            struct coordinate_type<Point6D>
            {
                using type = uint32_t;
            };

            template <>
            struct coordinate_system<Point6D>
            {
                using type = cs::cartesian;
            };

            template <>
            struct dimension<Point6D> : boost::mpl::int_<D>
            {
            };

            template <std::size_t Dimension>
            struct access<Point6D, Dimension>
            {
                static inline uint32_t get(Point6D const &p)
                {
                    return p[Dimension];
                }

                static inline void set(Point6D &p, uint32_t const &value)
                {
                    p[Dimension] = value;
                }
            };

        }
    }
}