#pragma once

#include "voxelizer.h"
#include <array>
#include <cstdio>
#include <vector>

void load_file(const std::string& filename);
int  export_magicavoxel(const char*          filename,
                        const unsigned char  grid[],
                        std::array<int, 3>   grid_size,
                        int                  voxels_n,
                        Voxelizer::VoxelMeta data[] = nullptr);
int export_raw(const unsigned char grid[], std::array<int, 3> grid_size);
