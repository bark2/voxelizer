#pragma once

#include <array>
#include <cassert>
#include <stddef.h>

namespace Voxelizer {

    enum { SUCCESS, ERROR_NO_DATA_BUFFER, ERROR_VOXEL_WITHOUT_A_MATCH };

enum VoxelType : unsigned char { NONE, CROUDED, CLOSING, OPENING, BOTH };

size_t size_of_voxel_type_with_collision();
size_t size_of_voxel_type();

char voxelize(unsigned char output_grid[],
              unsigned int* voxel_count,
              int           grid_size_x,
              int           grid_size_y,
              int           grid_size_z,
              double (*meshes[])[3][3],
              size_t        meshes_size[],
              size_t        meshes_count,
              bool          flip_normals            = false,
              double        triangles_min[3]        = nullptr,
              double        triangles_max[3]        = nullptr,
              bool          flood_fill              = false,
              bool          use_collision_detection = false,
              unsigned char metadata[]              = nullptr);
}
