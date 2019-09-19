#pragma once

#include "csignal"
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

struct Voxel_ {
    bool valid;
    enum Type { NONE, CLOSING, OPENING, BOTH } max_type;
    f64 max_coll_off;
};

// struct VoxelMeta {
    // u32 idx;
    // enum Type { CROUDED, CLOSING, OPENING, BOTH } type;
// };

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

}
