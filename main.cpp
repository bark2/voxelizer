#include "common.h"
#include "iritSkel.h"
#include "math.h"
#include "obj.h"
#include "types.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

i32 at(array<i32, 3> grid_size, i32 x, i32 y, i32 z)
{
    return x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
}

void
flood_fill_rec(
    std::vector<Voxel>& grid, const array<i32, 3>& grid_size, u32& voxels_n, i32 x, i32 y, i32 z)
{
    if (x < 0 || y < 0 || z < 0 || x >= grid_size.at(0) || y >= grid_size.at(1) || z >= grid_size.at(2))
        return;
    if (grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z).valid) return;

    grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z).valid = true;
    voxels_n++;

    flood_fill_rec(grid, grid_size, voxels_n, x, y, z + 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y, z - 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y + 1, z);
    flood_fill_rec(grid, grid_size, voxels_n, x, y - 1, z);
    flood_fill_rec(grid, grid_size, voxels_n, x + 1, y, z);
    flood_fill_rec(grid, grid_size, voxels_n, x - 1, y, z);
}

void
flood_fill_rast(Array& grid, const array<i32, 3>& grid_size, u32& voxels_n)
{
    struct {
        Voxel::Type type;
        i32 z;
    } last;

    for (i32 x = 0; x < grid_size[0]; x++) {
        for (i32 y = 0; y < grid_size[1]; y++) {
            last = { Voxel::CLOSING, std::numeric_limits<i32>::max() };
            for (i32 z = 0; z < grid_size[2]; z++) {
                Voxel* voxel =
                    static_cast<Voxel*>(grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z));
                if (voxel->valid) { // NONE || BOTH || CLOSING
                    last = { voxel->max_type, z };
                } else if (last.type == Voxel::OPENING && z > last.z) {
                    voxel->valid = true;
                    voxels_n++;
                }
            }

            if (last.type == Voxel::OPENING) printf("bad point: %d %d %d\n", x, y, last.z);
        }
    }
}

void
grid_cut(std::vector<Voxel>& grid,
         const array<i32, 3>& grid_size,
         u32& voxels_n,
         const array<i32, 3>& threshold)
{
    for (i32 x = 0; x < grid_size[0]; x++) {
        for (i32 y = 0; y < grid_size[1]; y++) {
            for (i32 z = 0; z < grid_size[2]; z++) {
                i32 at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                if (x >= threshold[0] && y >= threshold[1] && z >= threshold[2] && grid.at(at).valid) {
                    grid.at(at).valid = false;
                    voxels_n--;
                }
            }
        }
    }
}

void
print_usage()
{
    printf(
        "Usage: vox --in file --grid z,x,y [--flood-fill] [--flip-normals] [--magicavoxel --include-normals]\n");
}

vec3 scene_aabb_min;
vec3 scene_aabb_max;
std::vector<Triangle> triangles;

int
main(int argc, char* argv[])
{
    assert(std::is_pod<vec3>::value && std::is_pod<vec2>::value);

    char filename[128] = {};
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

    array<i32, 3> grid_size;
    char** arg_resolution = get_cmd(argv, argv + argc, "--grid");
    if (!arg_resolution) {
        printf("Input Error: wrong grid format");
        print_usage();
        return 1;
    }
    sscanf(arg_resolution[1], "%u,%u,%u", &grid_size[2], &grid_size[0], &grid_size[1]);

    bool do_include_normals = false;
    if (get_cmd(argv, argv + argc, "--include-normals")) do_include_normals = true;

    bool do_flood_fill = false;
    if (get_cmd(argv, argv + argc, "--flood-fill")) {
        do_flood_fill = true;
        do_include_normals = true;
    }

    bool do_flip_normals = false;
    if (get_cmd(argv, argv + argc, "--flip-normals")) do_flip_normals = true;

    enum class FORMAT { RAW, MAGICAVOXEL } do_export = FORMAT::RAW;
    char** arg_format = get_cmd(argv, argv + argc, "--magicavoxel");
    if (arg_format) do_export = FORMAT::MAGICAVOXEL;

    Array grid;
    if (!do_include_normals)
        grid.buff = static_cast<u8*>(calloc(grid_size[0] * grid_size[1] * grid_size[2] / 8, 1));
    else {
        grid.buff = static_cast<u8*>(calloc(grid_size[0] * grid_size[1] * grid_size[2], sizeof(Voxel)));
        grid.elem_bits = sizeof(Voxel) * 8;
    }
    if (!grid.buff) {
        perror("Error calloc");
        return 1;
    }

    scene_aabb_min = { std::numeric_limits<f64>::max(), std::numeric_limits<f64>::max(),
                       std::numeric_limits<f64>::max() };
    scene_aabb_max = { -std::numeric_limits<f64>::max(), -std::numeric_limits<f64>::max(),
                       -std::numeric_limits<f64>::max() };
    if (strstr(filename, ".itd")) {
        if (!CGSkelProcessIritDataFiles((const char*)filename)) {
            printf("Error: irit parser\n");
            return 1;
        }
    } else {
        const auto meshes = load_file(filename);
        std::size_t triangles_num = 0;
        for (auto& mesh : meshes) triangles_num += mesh.vertices.size() / 3;
        triangles.reserve(triangles_num);
        for (auto& mesh : meshes) {
            for (size_t i = 0; i < mesh.indices.size(); i += 3) {
                Triangle t;
                for (i32 j = 0; j < 3; j++) {
                    for (i32 k = 0; k < 3; k++) {
                        t[k][j] = mesh.vertices[mesh.indices[i + k]].pos[j];
                        scene_aabb_max[j] =
                            std::max(scene_aabb_max[j], mesh.vertices[mesh.indices[i + k]].pos[j]);
                        scene_aabb_min[j] =
                            std::min(scene_aabb_min[j], mesh.vertices[mesh.indices[i + k]].pos[j]);
                    }
                }
                triangles.emplace_back(t);
            }
        }
    }

    grid_size = swizzle(grid_size, 2);
    scene_aabb_max = swizzle(scene_aabb_max, 2);
    scene_aabb_min = swizzle(scene_aabb_min, 2);
    f64 scene_aabb_min_min = std::min({ scene_aabb_min[0], scene_aabb_min[1], scene_aabb_min[2] });
    f64 scene_aabb_max_max = std::max({ scene_aabb_max[0], scene_aabb_max[1], scene_aabb_max[2] });

    for (auto& t : triangles) {
        if (do_flip_normals) std::swap(t[1], t[2]);
        for (auto& v : t) {
            v = swizzle(v, 2);
            // saves the headache of finding the grid voxel for each point
            for (i32 i = 0; i < 3; i++) {
                // normalize to[0.0f, grid_size - 1.0f]
                v[i] = (v[i] - scene_aabb_min[i]) / (scene_aabb_max_max - scene_aabb_min_min);
                v[i] *= grid_size[i] - 1.0f;
            }
        }
    }

    u32 voxels_n = 0;
    array<u32, 3> mesh_center {};
    for (auto& t : triangles) {
        vec3 tmin, tmax;
        triangle_aabb(t, &tmin, &tmax);
        array<i32, 3> aligned_min, aligned_max;
        for (int i = 0; i < 3; i++) {
            aligned_min[i] = static_cast<i32>(progressive_floor(tmin[i]));
            aligned_max[i] = static_cast<i32>(std::floor(tmax[i]));
        }

        vec3 tri_normal = cross(t[1] - t[0], t[2] - t[0]);
        vec3 edges[3];
        for (int i = 0; i < 3; i++) edges[i] = t[(i + 1) % 3] - t[i];

        for (i32 x = aligned_min[0]; x <= aligned_max[0]; x++) {
            for (i32 y = aligned_min[1]; y <= aligned_max[1]; y++) {
                for (i32 z = aligned_min[2]; z <= aligned_max[2]; z++) {
                    auto at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                    vec3 min_voxel = vec3(x, y, z);
                    array<vec3, 2> aabb = { min_voxel, min_voxel + 1.0f };

                    bool is_coll = triangle_aabb_collision_mt(t, tri_normal, edges, aabb);
                    if (is_coll) {
                        if (!do_include_normals) {
                            u8* voxels = static_cast<u8*>(grid.at(at));
                            u8 bit_number = z % 8;
                            u8 mask = 1 << bit_number;
                            if (!(*voxels & mask)) {
                                *voxels = *voxels | mask;
                                voxels_n++;
                            }
                        } else {
                            Voxel* voxel = static_cast<Voxel*>(grid.at(at));
                            if (!voxel->valid) {
                                voxel->valid = true;
                                voxels_n++;
                            }

                            auto colls = find_triangle_aabb_collision(t, edges, aabb);
                            if (colls.empty()) continue; // the triangle is inside the aabb

                            auto max_coll = *std::max_element(colls.cbegin(), colls.cend(),
                                                              [](const vec3& l, const vec3& r) {
                                                                  return l.z < r.z;
                                                              });

                            Voxel::Type type;
                            if (std::abs(tri_normal.z) < epsilon)
                                type = Voxel::BOTH;
                            else if (tri_normal.z < 0.0f)
                                type = Voxel::OPENING;
                            else if (tri_normal.z > 0.0f)
                                type = Voxel::CLOSING;

                            if (max_coll.z + epsilon < voxel->max_coll_off) continue;
                            if (voxel->max_type == Voxel::NONE || max_coll.z > voxel->max_coll_off) {
                                voxel->max_type = type;
                                voxel->max_coll_off = max_coll.z;
                            } else if (type == Voxel::CLOSING) { // epsilon included
                                voxel->max_type = type;
                                voxel->max_coll_off = max_coll.z;
                            }
                        }
                    }
                }
            }
        }
    }

    if (do_flood_fill) flood_fill_rast(grid, grid_size, voxels_n);

    if (do_export == FORMAT::MAGICAVOXEL) {
        assert(std::all_of(grid_size.cbegin(), grid_size.cend(), [](const int& x) { return x < 129; }));
        if (export_magicavoxel("bunny.vox", grid, grid_size, voxels_n, do_include_normals))
            assert(0 && "couldn't not open the vox file");
        printf("voxels:\t%u\n", voxels_n);
    } else
        export_raw(grid, grid_size, do_include_normals);

    return 0;
}
