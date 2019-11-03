#pragma once

#include <array>
#include <cassert>
#include <stddef.h>

namespace Voxelizer {

enum ReturnType { SUCCESS, ERROR_NO_DATA_BUFFER, ERROR_VOXEL_WITHOUT_A_MATCH };
enum FillType { FILL_NONE, FILL_FLOOD, FILL_FLOOD_MESHES, FILL_SCANLINE };

enum VoxelType : unsigned char { NONE, CROUDED, CLOSING, OPENING, BOTH };

size_t size_of_voxel_type_presice();
size_t size_of_voxel_type();
size_t size_of_voxel_type_flood_fill();

/*
triangle_min\max:       if not provided, will be calculated be the function

voxel_types:            must be alocated to size (grid_size * size_of_voxel_type()) or 
                        (grid_size * size_of_voxel_type_precise()), if precise option is used
                        must be provided when using FILL_SCANLINE option

flood_fill_mem:         must be alocated to size (grid_size * size_of_voxel_type_flood_fill()) if FLOOD_FILL is used
 */

char voxelize(unsigned char output_grid[],  // output bitfield grid, should be in size of grid_size/8
              unsigned int* voxel_count,    // number of ouput voxels
              int           grid_size_x,    // grid size x coordinate
              int           grid_size_y,    // grid size y coordinate
              int           grid_size_z,    // grid size z coordinate
              double (*meshes[])[3][3],     // an array of pointers to the triangular meshes
              unsigned char flip_normals[], // should the triangles be treated as front faced
              size_t        meshes_size[],  // sizes of the meshes
              size_t        meshes_count,   // the size of the meshes pointers array
              double        triangles_min[3] = nullptr,   // the min for all meshes for each coordinate
              double        triangles_max[3] = nullptr,   // the max for all meshes for each coordinate
              FillType      fill             = FILL_NONE, // fill type
              unsigned char flood_fill_mem[] = nullptr,   // used by FLOOD_FILL
              bool          precise = false, // should use collision detection function to output voxels
              unsigned char voxel_types[] = nullptr, // output voxel types if specified non null
              bool          verbose       = false);
}
