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

// #ifdef AI
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

using std::array;
using Triangle = array<array<double, 3>, 3>;

void
processMesh(aiMesh* mesh)
{
    extern double                scene_aabb_min[3];
    extern double                scene_aabb_max[3];
    extern std::vector<Triangle> triangles;

    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        assert(face.mNumIndices == 3);

        Triangle t;
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                t[j][k]           = mesh->mVertices[face.mIndices[j]][k];
                scene_aabb_min[k] = std::min(scene_aabb_min[k], t[j][k]);
                scene_aabb_max[k] = std::max(scene_aabb_max[k], t[j][k]);
            }
        }

        triangles.push_back(t);
    }
}

void
processNode(aiNode* node, const aiScene* scene)
{
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        processMesh(mesh);
    }

    for (unsigned int i = 0; i < node->mNumChildren; i++) processNode(node->mChildren[i], scene);
}

void
ai_load_file(const std::string& filename)
{
    Assimp::Importer importer;
    const aiScene*   scene = importer.ReadFile(filename, aiProcess_Triangulate);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        printf("ERROR::ASSIMP::%s", importer.GetErrorString());
        return;
    }

    processNode(scene->mRootNode, scene);
}

//#endif

static const unsigned int max_line = 128;

enum Command { POSITION = 0, TEXTURE, NORMAL, FACE, GROUP, OBJECT, SMOOTH, COMMENT, COMMANDS_COUNT };

static const std::array<const std::string, COMMANDS_COUNT> commands_map { "v ", "vt ", "vn ", "f ",
                                                                          "g ", "o ",  "s ",  "#" };

void
load_file(const std::string& filename)
{
    ai_load_file(filename);
    return;
}

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
export_magicavoxel(const char*          filename,
                   const unsigned char  grid[],
                   array<int, 3>        grid_size,
                   int                  voxels_n,
                   Voxelizer::VoxelMeta data[])
{
    FILE* out = fopen(filename, "wb");
    if (!out) return 1;

    unsigned int  meta_size     = 3 * sizeof(unsigned int);
    unsigned int  size_size     = 3 * sizeof(unsigned int);
    int           max_grid_size = std::max({ grid_size[0], grid_size[1], grid_size[2] });
    array<int, 3> scaling       = { max_grid_size / grid_size[0], max_grid_size / grid_size[1],
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
                                if (data) {
                                    auto type = data[voxel_num].type;
                                    switch (type) {
                                    case Voxelizer::VoxelMeta::OPENING: color = 122; break;
                                    case Voxelizer::VoxelMeta::CLOSING: color = 218; break;
                                    case Voxelizer::VoxelMeta::CROUDED: color = 7 << 5; break;
                                    default: break;
                                    }
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

    assert(static_cast<int>(output_voxels) == voxels_n * scaling[0] * scaling[1] * scaling[2]);
    return 0;
}

int
export_raw(const unsigned char grid[], array<int, 3> grid_size)
{
    for (int z = 0; z < grid_size[0]; z++)
        for (int x = 0; x < grid_size[1]; x++)
            for (int y = 0; y < grid_size[2]; y++) {
                unsigned int         at = (z * grid_size[1] * grid_size[2] + x * grid_size[2] + y);
                bool                 is_valid;
                const unsigned char* voxels     = &grid[at / 8];
                unsigned char        bit_number = at % 8;
                unsigned char        mask       = 1 << bit_number;
                is_valid                        = *voxels & mask;

                if (is_valid)
                    puts("1");
                else
                    puts("0");
            }
    return 0;
}
