#pragma once

#include <iostream>
#include <array>
#include <cstdint>
#include <cmath>
#include <utility>
#include "tlx/container/btree_multimap.hpp"

template <size_t D>
using Point = std::array<uint32_t, D>;

template <size_t D>
__int128_t morton_encoding(const Point<D> &p)
{
    __int128_t morton_key = 0;
    const int bits_per_dim = 16;

    for (int i = 0; i < bits_per_dim; i++)
    {
        for (size_t j = 0; j < D; j++)
        {

            __int128_t bit = (p[j] >> i) & 1;
            morton_key |= bit << (i * D + j);
        }
    }
    return morton_key;
}

template <size_t D>
__int128_t distance_squared(const Point<D> &p1, const Point<D> &p2)
{
    __int128_t sq_dis = 0;
    for (size_t i = 0; i < D; i++)
    {
        __int128_t diff = (__int128_t)p1[i] - (__int128_t)p2[i];
        sq_dis += diff * diff;
    }
    return sq_dis;
}

template <size_t D>
class SpatialTree
{
private:
    tlx::btree_multimap<__int128_t, Point<D>> tree;

public:
    void insert(const Point<D> &point)
    {
        __int128_t key = morton_encoding<D>(point);
        tree.insert({key, point});
    }

    template <typename VisitorFunc>
    void query_radius(const Point<D> &center, double radius, VisitorFunc visitor)
    {
        __int128_t center_key = morton_encoding<D>(center);
        __int128_t delta = 500000;
        __int128_t start_key = center_key > delta ? center_key - delta : 0;
        __int128_t end_key = center_key + delta;
        __int128_t radius_sq = (__int128_t)(radius * radius);

        auto it_start = tree.lower_bound(start_key);

        auto it_end = tree.upper_bound(end_key);

        for (auto it = it_start; it != it_end; ++it)
        {
            const Point<D> &candidate_point = it->second;
            if (distance_squared<D>(center, candidate_point) <= radius_sq)
            {
                visitor(candidate_point);
            }
        }
    }
};