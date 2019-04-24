#pragma once

#include "types.h"
#include <cstdio>

std::vector<Mesh> load_obj_file(const std::string& filename);
std::vector<Mesh> load_obj_file_(const std::string& filename);
int export_vox_file(
    const std::string& filename, const std::vector<bool>& grid, u32 resolution, u32 voxels_n);
