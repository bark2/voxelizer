#include "common.h"
#include "iritSkel.h"
#include "voxelizer.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

using std::array;
static int export_magicavoxel(const char*          filename,
                              const unsigned char  grid[],
                              array<int, 3>        grid_size,
                              int                  voxels_n,
                              Voxelizer::VoxelMeta data[]);

void
print_usage()
{
    printf(
        "Usage: vox --in file --grid z,x,y [--flood-fill] [--flip-normals] [--magicavoxel --include-normals]\n");
}

float scene_aabb_min[3] = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max() };
float scene_aabb_max[3] = { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                            -std::numeric_limits<float>::max() };
using Triangle          = std::array<std::array<float, 3>, 3>;
std::vector<Triangle> triangles;
unsigned int          qtri = 1173;

int
main()
{
    array<int, 3>  grid_size = { 128, 128, 128 };
    unsigned char* grid1     = (unsigned char*)calloc(grid_size[0] * grid_size[1] * grid_size[2] / 8, 1);
    unsigned char* grid2     = (unsigned char*)calloc(grid_size[0] * grid_size[1] * grid_size[2] / 8, 1);

    Voxelizer::VoxelMeta* data1 =
        (Voxelizer::VoxelMeta*)calloc(grid_size[0] * grid_size[1] * grid_size[2],
                                      sizeof(Voxelizer::VoxelMeta));
    Voxelizer::VoxelMeta* data2 =
        (Voxelizer::VoxelMeta*)calloc(grid_size[0] * grid_size[1] * grid_size[2],
                                      sizeof(Voxelizer::VoxelMeta));

    static char filename[] = "tri.vox";
    if (!CGSkelProcessIritDataFiles((const char*)"./data/MediumSizeFiles/babem.itd")) {
        printf("Error: irit parser\n");
        return 1;
    }
    std::vector<Triangle> triangles_tmp;
    triangles_tmp.reserve(triangles.size());
    float(*meshes[])[3][3] = { (float(*)[3][3])triangles_tmp.data() };
    size_t triangle_count  = triangles.size();

    printf("triangle[0]:\n");
    std::copy(std::begin(triangles), std::end(triangles), std::begin(triangles_tmp));
    int voxel_count = Voxelizer::voxelize(grid1, grid_size[0], grid_size[1], grid_size[2], meshes,
                                          &triangle_count, 1, false, nullptr, nullptr, false, data1);
    printf("voxels:\t%d\n", voxel_count);
    assert(std::all_of(grid_size.cbegin(), grid_size.cend(), [](const int& x) { return x < 129; }));
    if (export_magicavoxel("tri-orig.vox", grid1, grid_size, voxel_count, data1))
        assert(0 && "couldn't not open the vox file");

    free(grid1);
    free(data1);
    return 0;
}

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
