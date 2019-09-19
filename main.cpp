#include "common.h"
#include "iritSkel.h"
#include "obj.h"
#include "voxelizer.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

using std::array;
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
using Triangle          = float[3][3];
std::vector<Triangle> triangles;

int
main(int argc, char* argv[])
{
    // assert(std::is_pod<vec3>::value && std::is_pod<vec2>::value);

    char   filename[128]  = {};
    char** arg_input_file = get_cmd(argv, argv + argc, "--in");
    if (!arg_input_file) {
        printf("Input Error: must specify input file\n");
        print_usage();
        return 1;
    }
    sscanf(arg_input_file[1], "%s", filename);
#ifndef AI
    if (strstr(filename, ".obj") && strstr(filename, ".itd")) {
        printf("Input Error: file extention isnt supported\n");
        return 1;
    }
#endif
    assert(strlen(filename) < IRIT_LINE_LEN_VLONG - 5);

    char   out_filename[128] = {};
    char** arg_output_file   = get_cmd(argv, argv + argc, "--out");
    if (arg_output_file)
        sscanf(arg_output_file[1], "%s", out_filename);
    else {
        char* start     = strrchr(filename, '/');
        start           = start ? start + 1 : filename;
        const char* dot = strrchr(filename, '.');
        memcpy(out_filename, start, dot - start);
        strcpy(out_filename + (dot - start), ".vox");
        // printf("%s\n", out_filename);
    }

    assert(strlen(out_filename) < IRIT_LINE_LEN_VLONG - 5);

    array<int, 3> grid_size;
    char**        arg_resolution = get_cmd(argv, argv + argc, "--grid");
    if (!arg_resolution) {
        printf("Input Error: wrong grid format");
        print_usage();
        return 1;
    }
    sscanf(arg_resolution[1], "%d,%d,%d", &grid_size[2], &grid_size[0], &grid_size[1]);

    bool include_normals = false;
    if (get_cmd(argv, argv + argc, "--include-normals")) include_normals = true;

    bool flood_fill = false;
    if (get_cmd(argv, argv + argc, "--flood-fill")) {
        flood_fill      = true;
        include_normals = true;
    }

    bool flip_normals = false;
    if (get_cmd(argv, argv + argc, "--flip-normals")) flip_normals = true;

    enum class FORMAT { RAW, MAGICAVOXEL } do_export = FORMAT::RAW;
    char** arg_format                                = get_cmd(argv, argv + argc, "--magicavoxel");
    if (arg_format) do_export = FORMAT::MAGICAVOXEL;

    if (strstr(filename, ".itd")) {
        if (!CGSkelProcessIritDataFiles((const char*)filename)) {
            printf("Error: irit parser\n");
            return 1;
        }
    } else {
        load_file(filename);
    }

    float(*meshes[])[3][3] = { (float(*)[3][3])triangles.data() };
    size_t triangle_count  = triangles.size();

    unsigned char* grid = (unsigned char*)calloc(grid_size[0] * grid_size[1] * grid_size[2] / 8, 1);
    if (!grid) { return 1; }

    Voxelizer::VoxelMeta* data =
        (Voxelizer::VoxelMeta*)calloc(grid_size[0] * grid_size[1] * grid_size[2],
                                      sizeof(Voxelizer::VoxelMeta));
    int voxel_count = Voxelizer::voxelize(grid, grid_size[0], grid_size[1], grid_size[2], meshes,
                                          &triangle_count, 1, flip_normals, (float*)&scene_aabb_min,
                                          (float*)&scene_aabb_max, flood_fill, data);

    if (do_export == FORMAT::RAW) {
        export_raw(grid, grid_size);
    } else {
        printf("voxels:\t%u\n", voxel_count);
        assert(std::all_of(grid_size.cbegin(), grid_size.cend(), [](const int& x) { return x < 129; }));
        if (export_magicavoxel(out_filename, grid, grid_size, voxel_count, data))
            assert(0 && "couldn't not open the vox file");
    }

    free(grid);
    free(data);
    return 0;
}
