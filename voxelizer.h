#pragma once

#include <array>
#include <cassert>
#include <stddef.h>

namespace Voxelizer {

template <unsigned N> struct IVoxelMeta {
    struct static_array {
        std::array<int, N> ids;
        size_t             count = 0;

        inline void
        push_back(int idx)
        {
            if (count != N) ids[count++] = idx;
        }

        inline bool
        is_empty()
        {
            return (ids == 0);
        }
    } ids;
    enum Type { CROUDED, CLOSING, OPENING, BOTH } type;
};
using VoxelMeta = IVoxelMeta<6>;

int voxelize(unsigned char output_grid[],
             int           grid_size_x,
             int           grid_size_y,
             int           grid_size_z,
             float (*meshes[])[3][3],
             size_t    meshes_size[],
             size_t    meshes_count,
             bool      flip_normals     = false,
             float     triangles_min[3] = nullptr,
             float     triangles_max[3] = nullptr,
             bool      flood_fill       = false,
             VoxelMeta metadata[]       = nullptr);
}
