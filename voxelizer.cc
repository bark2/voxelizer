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

bool _gdb = false;

array<vec2, 3>
to_vec2_array(const array<vec3, 3>& a)
{
    return { a[0], a[1], a[2] };
}

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
flood_fill_rast(std::vector<Voxel>& grid, const array<i32, 3>& grid_size, u32& voxels_n)
{
    struct {
        Voxel::Type type;
        i32 z;
    } last;

    for (i32 x = 0; x < grid_size[0]; x++) {
        for (i32 y = 0; y < grid_size[1]; y++) {
            last = { Voxel::CLOSING, std::numeric_limits<i32>::max() };
            for (i32 z = 0; z < grid_size[2]; z++) {
                auto& voxel = grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z);
                if (voxel.valid) { // NONE || BOTH || CLOSING
                    last = { voxel.max_type, z };
                } else if (last.type == Voxel::OPENING && z > last.z) {
                    voxel.valid = true;
                    voxels_n++;
                }
            }

            if (last.type == Voxel::OPENING) printf("bad point: %d %d %d\n", x, y, last.z);
        }
    }
}

void
flood_fill_inv(std::vector<Voxel>& grid, const array<i32, 3>& grid_size, u32& voxels_n)
{
    enum RType { NO_VOX_CLM, VOX_CLM };
    auto fill_dr = [&](i32 d1, i32 d2, i32 d3, bool dr) {
        for (i32 x = 0; x < grid_size[d1]; x++) {
            for (i32 y = 0; y < grid_size[d2]; y++) {
                for (i32 z = dr ? 0 : grid_size[d3] - 1; z >= 0 && z < grid_size[d3]; z += dr ? 1 : -1) {
                    if (!grid[at(grid_size, x, y, z)].valid) {
                        grid[at(grid_size, x, y, z)].valid = true;
                        voxels_n++;
                    } else if (z == grid_size[d3] - 1)
                        return NO_VOX_CLM;
                    else
                        return VOX_CLM;
                }
            }
        }
        return NO_VOX_CLM;
    };
    if (fill_dr(0, 1, 2, true) == VOX_CLM) fill_dr(0, 1, 2, false);
    if (fill_dr(2, 0, 1, true) == VOX_CLM) fill_dr(2, 0, 1, false);
    if (fill_dr(1, 2, 0, true) == VOX_CLM) fill_dr(1, 2, 0, false);

    voxels_n = grid_size[0] * grid_size[1] * grid_size[2] - voxels_n;
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

vec3 scene_aabb_min;
vec3 scene_aabb_max;
std::vector<Triangle> triangles;

int
main(int argc, char* argv[])
{
    array<i32, 3> grid_size;
    char** arg_resolution = get_cmd(argv, argv + argc, "--resolution");
    if (arg_resolution)
        sscanf(arg_resolution[1], "%u,%u,%u", &grid_size[2], &grid_size[0], &grid_size[1]);
    else
        assert(0 && "wrong grid resolution input");

    bool do_flood_fill_rast = false;
    if (get_cmd(argv, argv + argc, "--flood-fill-rast")) do_flood_fill_rast = true;
    bool do_flood_fill_rec = false;
    if (get_cmd(argv, argv + argc, "--flood-fill-rec")) do_flood_fill_rec = true;
    bool do_flood_fill_inv = false;
    if (get_cmd(argv, argv + argc, "--flood-fill-inv")) do_flood_fill_inv = true;

    enum class FORMAT { NONE, RAW, MAGICAVOXEL } do_export;
    char** arg_format = get_cmd(argv, argv + argc, "--export");
    if (arg_format && !strcmp(arg_format[1], "raw")) {
        do_export = FORMAT::RAW;
    } else if (arg_format && !strcmp(arg_format[1], "magicavoxel")) {
        do_export = FORMAT::MAGICAVOXEL;
    } else if (arg_format) {
        printf("Input Error: export\n");
        return 1;
    }

    std::vector<Voxel> grid(grid_size[0] * grid_size[1] * grid_size[2], { false, Voxel::NONE, 0.0f });

    scene_aabb_min = { std::numeric_limits<f32>::max(), std::numeric_limits<f32>::max(),
                       std::numeric_limits<f32>::max() };
    scene_aabb_max = { -std::numeric_limits<f32>::max(), -std::numeric_limits<f32>::max(),
                       -std::numeric_limits<f32>::max() };
    if (false) {
        // const char* file_name = "data/BasicModels/cube.itd";
        const char* file_name = "data/cow.itd";
        CGSkelProcessIritDataFiles((const char* const*)&file_name, 1);
    } else {
        const auto meshes = ai_load_obj_file("data/lowpolydeer/deer.obj");
        // const auto meshes = ai_load_obj_file("data/kitten.obj");
        // const auto meshes = ai_load_obj_file("data/bunny.obj");
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
    f32 scene_aabb_min_min = std::min({ scene_aabb_min[0], scene_aabb_min[1], scene_aabb_min[2] });
    f32 scene_aabb_max_max = std::max({ scene_aabb_max[0], scene_aabb_max[1], scene_aabb_max[2] });
    vec3 voxel_size = { (scene_aabb_max[0] - scene_aabb_min[0]) / grid_size[0],
                        (scene_aabb_max[1] - scene_aabb_min[1]) / grid_size[1],
                        (scene_aabb_max[2] - scene_aabb_min[2]) / grid_size[2] };

    // FIXME: slow!
    for (auto& t : triangles) {
        for (auto& v : t) {
            v = swizzle(v, 2);
            for (i32 i = 0; i < 3; i++)
                v[i] = (grid_size[i] - 1.0f) * (v[i] - scene_aabb_min[i]) /
                       (scene_aabb_max_max - scene_aabb_min_min);
        }
    }

    u32 voxels_n = 0;
    array<u32, 3> mesh_center {};
    for (auto& t : triangles) {
        vec3 triangle_normal = normal(t);
        array<bool, 3> back_facing;
        for (int i = 0; i < 3; i++) back_facing[i] = triangle_normal[i] > 0;

        vec3 tmin, tmax;
        triangle_aabb(t, &tmin, &tmax);
        array<i32, 3> aligned_min = { static_cast<i32>(progressive_floor(tmin.x)),
                                      static_cast<i32>(progressive_floor(tmin.y)),
                                      static_cast<i32>(progressive_floor(tmin.z)) };

        array<i32, 3> aligned_max = {
            static_cast<i32>(std::floor(tmax.x)),
            static_cast<i32>(std::floor(tmax.y)),
            static_cast<i32>(std::floor(tmax.z)),
        };

        for (i32 x = aligned_min[0]; x <= aligned_max[0]; x++) {
            for (i32 y = aligned_min[1]; y <= aligned_max[1]; y++) {
                for (i32 z = aligned_min[2]; z <= aligned_max[2]; z++) {
                    vec3 min_voxel = vec3(x, y, z);
                    array<vec3, 2> aabb = { min_voxel, min_voxel + 1.0f };
                    auto option = Intersection_Option::RETURN_INTERSECTOIN_POINT;
                    auto intersecting = triangle_aabb_collision(t, aabb, option);

                    if (intersecting.first && option == Intersection_Option::RETURN_INTERSECTOIN_POINT) {
                        auto& voxel = grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z);
                        // assert(
                            // (intersecting.second >= 0.0f && intersecting.second <= grid_size[2] - 1.0f));
                        if (intersecting.second < voxel.max_intersection_off) continue;

                        Voxel::Type type;
                        if (triangle_normal.z == 0.0f)
                            type = Voxel::BOTH;
                        else if (triangle_normal.z < 0.0f)
                            type = Voxel::OPENING;
                        else if (triangle_normal.z > 0.0f)
                            type = Voxel::CLOSING;

                        if (!voxel.valid) {
                            voxel.valid = true;
                            voxels_n++;
                            mesh_center[0] += x;
                            mesh_center[1] += y;
                            mesh_center[2] += z;
                            voxel.max_intersection_off = intersecting.second;
                            voxel.max_type = type;
                        } else if (intersecting.second > voxel.max_intersection_off) {
                            voxel.max_type = type;
                            voxel.max_intersection_off = intersecting.second;
                        } else if (intersecting.second == voxel.max_intersection_off &&
                                   type == Voxel::CLOSING) {
                            voxel.max_type = type;
                        }
                    }
                }
            }
        }
    }

    mesh_center = { mesh_center[0] / voxels_n, mesh_center[1] / voxels_n, mesh_center[2] / voxels_n };

    if (do_flood_fill_rast)
        flood_fill_rast(grid, grid_size, voxels_n);
    else if (do_flood_fill_rec)
        flood_fill_rec(grid, grid_size, voxels_n, mesh_center[0], mesh_center[1], mesh_center[2]);
    else if (do_flood_fill_inv)
        flood_fill_inv(grid, grid_size, voxels_n);

    if (do_export == FORMAT::MAGICAVOXEL) {
        if (export_magicavoxel("bunny.vox", grid, grid_size, voxels_n))
            assert(0 && "couldn't not open the vox file");
        printf("res:\t%u %u %u\n", grid_size[0], grid_size[1], grid_size[2]);
        printf("voxels:\t%u\n", voxels_n);
        if (do_flood_fill_rec)
            printf("mesh center:\t%u %u %u\n", mesh_center[0], mesh_center[1], mesh_center[2]);
    } else if (do_export == FORMAT::RAW) {
        export_raw(grid, grid_size);
    }

    return 0;
}
