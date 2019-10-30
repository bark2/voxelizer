#pragma once

#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "vec2.h"
#include "vec3.h"

namespace IVoxelizer {

using u8  = unsigned char;
using i8  = int8_t;
using u32 = uint32_t;
using i32 = int32_t;

using std::array;
using std::vector;

using Triangle = array<vec3, 3>;

struct VoxelData {
    f64 max_coll_off;
    enum Type { NONE = 0, CLOSING, OPENING, BOTH } max_type;
};

inline void
set_voxel(u8* base, size_t voxel_num, bool value = true)
{
    if (value)
        base[voxel_num / 8] |= 1 << (voxel_num % 8);
    else
        base[voxel_num / 8] &= ~(1 << (voxel_num % 8));
}

inline bool
get_voxel(const u8* base, size_t voxel_num)
{
    u8     bit  = voxel_num % 8;
    size_t byte = voxel_num / 8;
    return base[byte] & (1 << bit);
}

inline void
set_voxel(u8* base, array<i32, 3> grid_size, array<i32, 3> v, bool value = true)
{
    set_voxel(base, v[0] * grid_size[1] * grid_size[2] + v[1] * grid_size[2] + v[2], value);
}

inline bool
get_voxel(const u8* base, array<i32, 3> grid_size, array<i32, 3> v)
{
    return get_voxel(base, v[0] * grid_size[1] * grid_size[2] + v[1] * grid_size[2] + v[2]);
}

struct Voxel_Queue {
    using Voxel = std::array<i32, 3>;

    Voxel*       base;
    unsigned int capacity;
    unsigned int tail = 0, head = 0;

    Voxel_Queue(Voxel* b, unsigned int c) : base(b), capacity(c) {}
    inline bool
    empty()
    {
        return tail == head;
    }

    inline void
    push(Voxel v)
    {
        assert((head + 1 != capacity && head + 1 != tail) || (head + 1 == capacity && tail != 0));
        base[head++] = v;
        if (head == capacity) head = 0;
    }

    inline Voxel
    pop()
    {
        assert(!empty());
        Voxel result = base[tail++];
        if (tail == capacity) tail = 0;
        return result;
    }

    inline Voxel
    front()
    {
        assert(!empty());
        return base[tail];
    }
};

}
