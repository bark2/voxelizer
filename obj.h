#pragma once

#include "types.h"
#include <cstdio>

std::vector<Mesh> load_obj_file(const std::string& filename);
std::vector<Mesh> ai_load_obj_file(const std::string& filename);
int export_magicavoxel(const std::string& filename,
                       const std::vector<Voxel>& grid,
                       array<i32, 3> grid_size,
                       u32 voxels_n);
int export_raw(const std::vector<Voxel>& grid, array<i32, 3> grid_size);
