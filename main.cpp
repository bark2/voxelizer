#include "vox_common.h"
#include "vox_iritSkel.h"
#include "vox_obj.h"
#include "voxelizer.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

using std::array;
using std::vector;
void
print_usage()
{
    printf(
        "voxelizer --in inputfile [--out outputfile] --grid x,y,z [--flood-fill or --flood-fill-meshes or --fill-scanline] [--precise] [--magicavoxel]\n");
}

using Triangle           = std::array<std::array<double, 3>, 3>;
double scene_aabb_min[3] = { std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::max() };
double scene_aabb_max[3] = { -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
                             -std::numeric_limits<double>::max() };

std::vector<std::vector<Triangle>> meshes;
Voxelizer::FillType                fill = Voxelizer::FILL_NONE;
std::vector<unsigned char>         is_flipped;

int
main(int argc, char* argv[])
{
    // assert(std::is_pod<vec3>::value && std::is_pod<vec2>::value);

    char   filename[128]  = {};
    char** arg_input_file = get_cmd(argv, argv + argc, "--in");
    if (!arg_input_file) {
        printf("Input Error: must specify input file\n");
        print_usage();
        fflush(stdout);
        return 1;
    }
    sscanf(arg_input_file[1], "%s", filename);
    if (!strstr(filename, ".itd")) {
        printf("Input Error: file extention isnt supported, only .itd files\n");
        fflush(stdout);
        return 1;
    }
    assert(strlen(filename) < IRIT_LINE_LEN_VLONG - 5);

    FILE*  out               = stdout;
    char   out_filename[128] = {};
    char** arg_output_file   = get_cmd(argv, argv + argc, "--out");
    if (arg_output_file) {
        sscanf(arg_output_file[1], "%s", out_filename);
        out = fopen(out_filename, "w+");
    }
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
    sscanf(arg_resolution[1], "%d,%d,%d", &grid_size[0], &grid_size[1], &grid_size[2]);

    unsigned char*      flood_fill_mem = nullptr;
    if (get_cmd(argv, argv + argc, "--flood-fill")) {
        fill           = Voxelizer::FILL_FLOOD;
        flood_fill_mem = (unsigned char*)malloc(grid_size[0] * grid_size[1] * grid_size[2] *
                                                Voxelizer::size_of_voxel_type_flood_fill());
    }
    if (get_cmd(argv, argv + argc, "--flood-fill-meshes")) {
        if (fill != Voxelizer::FILL_NONE) {
            printf("Error: only one fill type is allowed\n");
            return 1;
        }

        fill           = Voxelizer::FILL_FLOOD_MESHES;
        flood_fill_mem = (unsigned char*)malloc(grid_size[0] * grid_size[1] * grid_size[2] *
                                                Voxelizer::size_of_voxel_type_flood_fill());
    }
    if (get_cmd(argv, argv + argc, "--fill-scanline")) {
        if (fill != Voxelizer::FILL_NONE) {
            printf("Error: only one fill type is allowed\n");
            return 1;
        }

        fill = Voxelizer::FILL_SCANLINE;
    }

    bool use_collision_detection = false;
    if (get_cmd(argv, argv + argc, "--precise")) use_collision_detection = true;

    bool verbose = false;
    if (get_cmd(argv, argv + argc, "--verbose")) verbose = true;

    enum class FORMAT { RAW, MAGICAVOXEL } do_export = FORMAT::RAW;
    char** arg_format                                = get_cmd(argv, argv + argc, "--magicavoxel");
    if (arg_format) do_export = FORMAT::MAGICAVOXEL;

    if (!CGSkelProcessIritDataFiles((const char*)filename)) {
        printf("Error: irit parser\n");
        return 1;
    }

    unsigned char* grid = (unsigned char*)calloc(grid_size[0] * grid_size[1] * grid_size[2] / 8, 1);
    if (!grid) return 1;

    unsigned char* data = nullptr;
    if (!use_collision_detection) {
        data = (unsigned char*)calloc(grid_size[0] * grid_size[1] * grid_size[2],
                                      Voxelizer::size_of_voxel_type());
    }
    else {
        data = (unsigned char*)calloc(grid_size[0] * grid_size[1] * grid_size[2],
                                      Voxelizer::size_of_voxel_type_presice());
    }

    std::vector<double(*)[3][3]> mesh_ptrs(meshes.size());
    std::vector<size_t>          mesh_sizes(meshes.size());
    for (size_t i = 0; i < meshes.size(); i++) {
        mesh_ptrs[i]  = (double(*)[3][3])meshes[i].data();
        mesh_sizes[i] = meshes[i].size();
    }
    unsigned int voxel_count;
    char         err = Voxelizer::voxelize(grid, &voxel_count, grid_size[0], grid_size[1], grid_size[2],
                                   mesh_ptrs.data(), is_flipped.data(), mesh_sizes.data(), meshes.size(),
                                   (double*)&scene_aabb_min, (double*)&scene_aabb_max, fill,
                                   flood_fill_mem, use_collision_detection, data, verbose);
    if (err != Voxelizer::SUCCESS) {
        free(grid);
        free(data);
        free(flood_fill_mem);
        return 1;
    }

    if (do_export == FORMAT::RAW) {
        if (out != stdout) printf("voxels:\t%u\n", voxel_count);
        export_raw(grid, grid_size, out);
    }
    else {
        printf("voxels:\t%u\n", voxel_count);
        assert(std::all_of(grid_size.cbegin(), grid_size.cend(), [](const int& x) { return x < 129; }));
        if (verbose) printf("exporting to %s\n", out_filename);
        if (export_magicavoxel(out_filename, grid, grid_size, voxel_count, use_collision_detection,
                               data))
            assert(0 && "couldn't not open the vox file");
    }

    free(grid);
    free(data);
    free(flood_fill_mem);

    return 0;
}
