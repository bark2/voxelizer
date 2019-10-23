#include "common.h"
#include "types.h"
#include "voxelizer.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using std::array;
using Triangle = array<array<double, 3>, 3>;

// 2. Chunk Structure
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 1x4      | char       | chunk id
// 4        | int        | num bytes of chunk content (N)
// 4        | int        | num bytes of children chunks (M)
// N        |            | chunk content
// M        |            | children chunks
// -------------------------------------------------------------------------------
// 5. Chunk id 'SIZE' : model size
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4        | int        | size x
// 4        | int        | size y
// 4        | int        | size z : gravity direction
// -------------------------------------------------------------------------------
// 6. Chunk id 'XYZI' : model voxels
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4        | int        | numVoxels (N)
// 4 x N    | int        | (x, y, z, colorIndex) : 1 byte for each component
// -------------------------------------------------------------------------------

inline unsigned int
to_little(unsigned int big)
{
    unsigned int c1 = (big & 0xFF) << 24;
    unsigned int c2 = (big & (0xFF << 8)) << 8;
    unsigned int c3 = (big & (0xFF << 16)) >> 8;
    unsigned int c4 = (big & (0xFF << 24)) >> 24;
    return c1 | c2 | c3 | c4;
}

enum Type { CROUDED, CLOSING, OPENING, BOTH };

// little indian
int
export_magicavoxel(const char*         filename,
                   const unsigned char grid[],
                   array<int, 3>       grid_size,
                   unsigned int        voxels_n,
                   bool                use_collision_detection,
                   unsigned char       data[])
{
    FILE* out = fopen(filename, "wb");
    if (!out) return 1;

    unsigned int meta_size     = 3 * sizeof(unsigned int);
    unsigned int size_size     = 3 * sizeof(unsigned int);
    int          max_grid_size = std::max({ grid_size[0], grid_size[1], grid_size[2] });
    grid_size                  = { grid_size[2], grid_size[0], grid_size[1] };
    array<int, 3> scaling      = { max_grid_size / grid_size[0], max_grid_size / grid_size[1],
                              max_grid_size / grid_size[2] };
    voxels_n *= scaling[0] * scaling[1] * scaling[2];
    unsigned int xyzi_size          = (voxels_n + 1) * sizeof(unsigned int);
    unsigned int header_size        = 56;
    unsigned int total              = header_size + xyzi_size;
    unsigned int main_children_size = total - meta_size - 8;

    unsigned int              header[] = { to_little(0x564f5820),
                              150,
                              to_little(0x4d41494e),
                              0,
                              main_children_size,
                              to_little(0x53495a45),
                              size_size,
                              0,
                              static_cast<unsigned int>(grid_size[0]),
                              static_cast<unsigned int>(grid_size[1]),
                              static_cast<unsigned int>(grid_size[2]),
                              to_little(0x58595a49),
                              xyzi_size,
                              0,
                              static_cast<unsigned int>(voxels_n) };
    std::vector<unsigned int> buffer;
    buffer.reserve(total);
    for (auto&& s : header) buffer.emplace_back(s);

    unsigned int output_voxels = 0;
    for (unsigned char x = 0; x < grid_size[0]; x++)
        for (unsigned char y = 0; y < grid_size[1]; y++)
            for (unsigned char z = 0; z < grid_size[2]; z++) {
                unsigned int voxel_num = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                if (IVoxelizer::get_voxel(grid, voxel_num)) {
                    for (unsigned char i = 0; i < scaling[0]; i++)
                        for (unsigned char j = 0; j < scaling[1]; j++)
                            for (unsigned char k = 0; k < scaling[2]; k++) {
                                unsigned char color = 1;
                                if (data && !use_collision_detection) {
                                    Voxelizer::VoxelType type = ((Voxelizer::VoxelType*)data)[voxel_num];
                                    switch (type) {
                                    case Voxelizer::VoxelType::OPENING: color = 122; break;
                                    case Voxelizer::VoxelType::CLOSING: color = 218; break;
                                    case Voxelizer::VoxelType::CROUDED: color = (7 << 5); break;
                                    default: break;
                                    }
                                }
                                if (data && use_collision_detection) {
                                    IVoxelizer::VoxelData::Type type =
                                        ((IVoxelizer::VoxelData*)data)[voxel_num].max_type;
                                    if (type == 0)
                                        color = 1;
                                    else if (type == 1)
                                        color = 218;
                                    else if (type == 2)
                                        color = 122;
                                    else if (type == 3)
                                        color = 7 << 5;
                                }
                                unsigned int position_color = ((x * scaling[0] + i) << 24) +
                                                              ((y * scaling[1] + j) << 16) +
                                                              ((z * scaling[2] + k) << 8) + color;
                                buffer.emplace_back(to_little(position_color));
                                output_voxels++;
                            }
                }
            }

    fwrite(buffer.data(), sizeof(unsigned int), buffer.size(), out);
    fclose(out);

    assert(output_voxels == voxels_n);
    return 0;
}

int
export_raw(const unsigned char grid[], array<int, 3> grid_size, FILE* out)
{
    for (int x = 0; x < grid_size[0]; x++)
        for (int y = 0; y < grid_size[1]; y++)
            for (int z = 0; z < grid_size[2]; z++) {
                unsigned int         at = (x * grid_size[1] * grid_size[2] + y * grid_size[2] + z);
                bool                 is_valid;
                const unsigned char* voxels     = &grid[at / 8];
                unsigned char        bit_number = at % 8;
                unsigned char        mask       = 1 << bit_number;
                is_valid                        = *voxels & mask;

                if (is_valid)
                    fputc('1', out);
                else
                    fputc('0', out);
            }
    return 0;
}
