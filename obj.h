#pragma once

#include "types.h"
#include <cstdio>

std::vector<Mesh> load_file(const std::string& filename);
int export_magicavoxel(const std::string& filename,
                       const Array& grid,
                       array<i32, 3> grid_size,
                       u32 voxels_n, bool is_normal_included);
int export_raw(const Array& grid, array<i32, 3> grid_size);
