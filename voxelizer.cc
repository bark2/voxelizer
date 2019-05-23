#include "obj.h"
#include "types.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <limits>
#include <tuple>
#include <utility>

array<Swizzler<vec3>, 3> swizzlers = { get_swizzler<vec3>(0), get_swizzler<vec3>(1), get_swizzler<vec3>(2) };

inline f32
signed_edge_function(const vec2& v0, const vec2& v1, const bool back_facing, const vec2& test_point)
{
    f32 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = v1 - v0;
    vec2 edge_normal = d * vec2(-edge.y, edge.x);
    return dot(edge_normal, test_point - v0);
}

inline bool
point_in_triangle(const array<vec2, 3>& proj_triangle, bool back_facing, const vec2& p)
{
    bool result = true;
    const array<array<vec2, 2>, 3> edges = { array<vec2, 2> { proj_triangle[0], proj_triangle[1] },
                                             { proj_triangle[1], proj_triangle[2] },
                                             { proj_triangle[2], proj_triangle[0] } };

    // for (auto& e : edges) result &= signed_edge_function(e, back_facing, p) >= 0;

    return result;
}

inline bool
triangle_aabb_conservative_collision(const array<vec2, 3>& proj_triangle, bool back_facing, const array<vec2, 2>& aabb)
{
    const array<vec2, 4> aabb_vertices = { aabb[0], aabb[1], { aabb[0].x, aabb[1].y }, { aabb[1].x, aabb[0].y } };
    for (auto& v : aabb_vertices)
        if (point_in_triangle(proj_triangle, back_facing, v)) return true;

    return false;
}

inline bool
triangle_aabb_6seperating_collision(const array<vec2, 3>& proj_triangle, bool back_facing, array<vec3, 2> aabb)
{
    bool result = true;
    for (auto projection : swizzlers) {
        vec2 aabb_min = projection(aabb[0]);
        vec2 aabb_max = projection(aabb[1]);
        const array<vec2, 4> means = { vec2 { (aabb[0].x + aabb[1].x) / 2, aabb[0].y },
                                       { (aabb[0].x + aabb[1].x) / 2, aabb[1].y },
                                       { aabb[0].x, (aabb[0].y + aabb[1].y) / 2 },
                                       { aabb[1].x, (aabb[0].y + aabb[1].y) / 2 } };
        bool proj_voxel_collision = false;
        for (auto& v : means) proj_voxel_collision |= point_in_triangle(proj_triangle, back_facing, v);
        result &= proj_voxel_collision;
    }
    return false;
}

inline bool
aabb_collision(const vec3& b1_min, const vec3& b1_max, const vec3& b2_min, const vec3& b2_max)
{
    bool result = false;
    if ((b2_max.x - b1_min.x) * (b2_min.x - b1_max.x) >= 0.0 || (b2_max.y - b1_min.y) * (b2_min.y - b1_max.y) >= 0.0 ||
        (b2_max.z - b1_min.z) * (b2_min.z - b1_max.z) >= 0.0)
        result = true;
    return result;
}

inline u32
dominant_axis(const vec3& tnormal, array<vec3, 3> axises)
{
    array<f32, 3> normal_projections = { std::abs(dot(axises[0], tnormal)), std::abs(dot(axises[1], tnormal)),
                                         std::abs(dot(axises[2], tnormal)) };
    auto max_index = std::max_element(normal_projections.cbegin(), normal_projections.cend());
    return max_index - normal_projections.cbegin();
}

inline void
triangle_aabb(const Triangle& t, vec3* min, vec3* max)
{
    *min = { std::min({ t[0].x, t[1].x, t[2].x }), std::min({ t[0].y, t[1].y, t[2].y }),
             std::min({ t[0].z, t[1].z, t[2].z }) };
    *max = { std::max({ t[0].x, t[1].x, t[2].x }), std::max({ t[0].y, t[1].y, t[2].y }),
             std::max({ t[0].z, t[1].z, t[2].z }) };
}

inline f32
progressive_floor(f32 f)
{
    return std::max(0.0f, f == std::floor(f) ? f - 1 : std::floor(f));
}
inline f32
progressive_ceil(f32 f, f32 max)
{
    return std::min(max, f == std::ceil(f) ? f + 1 : std::ceil(f));
}

array<vec2, 3>
to_vec2_array(const array<vec3, 3>& a)
{
    return { a[0], a[1], a[2] };
}

char**
get_cmd(char** begin, char** end, const std::string& option)
{
    char** itr = std::find(begin, end, option);
    if (itr != end) return itr;
    return nullptr;
}

void
flood_fill_rec(std::vector<Voxel>& grid, const array<u32, 3>& grid_size, u32& voxels_n, u32 x, u32 y, u32 z)
{
    if (x < 0 || x >= grid_size.at(0) || y < 0 || y >= grid_size.at(1) || z < 0 || z >= grid_size.at(2)) return;
    if (grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z).valid) return;

    grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z).valid = true;
    voxels_n++;

    flood_fill_rec(grid, grid_size, voxels_n, x, y, z + 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y, z - 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y, z - 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y + 1, z);
    flood_fill_rec(grid, grid_size, voxels_n, x, y - 1, z);
    flood_fill_rec(grid, grid_size, voxels_n, x + 1, y, z);
    flood_fill_rec(grid, grid_size, voxels_n, x - 1, y, z);
}

// TODO: read IRIT data
// TODO: check the bloodfill

int
main(int argc, char* argv[])
{
    bool export_magicavoxel;
    array<u32, 3> grid_size;
    char** arg_resolution = get_cmd(argv, argv + argc, "--resolution");
    if (arg_resolution)
        sscanf(arg_resolution[1], "%u,%u,%u", &grid_size[2], &grid_size[0], &grid_size[1]);
    else
        assert(0 && "wrong grid resolution input");

    bool flood_fill_rast = false; // scanline method
    if (get_cmd(argv, argv + argc, "--flood-fill-rast")) flood_fill_rast = true;
    bool flood_fill = false;
    if (get_cmd(argv, argv + argc, "--flood-fill")) flood_fill = true;

    char** arg_format = get_cmd(argv, argv + argc, "--magicavoxel");
    if (arg_format) export_magicavoxel = true;

    // std::vector<bool> grid(grid_size[0] * grid_size[1] * grid_size[2], false);
    std::vector<Voxel> grid(grid_size[0] * grid_size[1] * grid_size[2], { false, Voxel::CLOSING });

    vec3 scene_aabb_min { std::numeric_limits<f32>::max() };
    vec3 scene_aabb_max { -std::numeric_limits<f32>::max() };
    // const auto meshes = ai_load_obj_file("data/lowpolydeer/deer.obj");
    const auto meshes = ai_load_obj_file("data/bunny.obj");
    // const auto meshes = ai_load_obj_file("data/kitten.obj");

    std::vector<Triangle> triangles;
    std::size_t triangles_num = 0;
    for (auto& mesh : meshes) triangles_num += mesh.vertices.size() / 3;
    triangles.reserve(triangles_num);
    for (auto& mesh : meshes) {
        for (u32 i = 0; i < mesh.indices.size(); i += 3) {
            Triangle t;
            for (u32 j = 0; j < 3; j++) {
                for (u32 k = 0; k < 3; k++) {
                    t[k][j] = mesh.vertices[mesh.indices[i + k]].pos[j];
                    scene_aabb_max[j] = std::max(scene_aabb_max[j], mesh.vertices[mesh.indices[i + k]].pos[j]);
                    scene_aabb_min[j] = std::min(scene_aabb_min[j], mesh.vertices[mesh.indices[i + k]].pos[j]);
                }
            }
            triangles.emplace_back(t);
        }
    }

    f32 scene_aabb_min_min = std::min({ scene_aabb_min[0], scene_aabb_min[1], scene_aabb_min[2] });
    f32 scene_aabb_max_max = std::max({ scene_aabb_max[0], scene_aabb_max[1], scene_aabb_max[2] });
    vec3 voxel_size = { (scene_aabb_max[0] - scene_aabb_min[0]) / grid_size[0],
                        (scene_aabb_max[1] - scene_aabb_min[1]) / grid_size[1],
                        (scene_aabb_max[2] - scene_aabb_min[2]) / grid_size[2] };

    for (auto& t : triangles)
        for (auto& v : t)
            for (u32 i = 0; i < 3; i++)
                v[i] = (grid_size[i] - 1.0f) * (v[i] - scene_aabb_min[i]) / (scene_aabb_max_max - scene_aabb_min_min);

    u32 voxels_n = 0;
    u32 more_than_one_triangle_in_voxel = 0;
    for (auto& t : triangles) {
        vec3 triangle_normal = t.normal();
        array<bool, 3> back_facing;
        for (int i = 0; i < 3; i++) back_facing[i] = triangle_normal[i] > 0;

        vec3 tmin, tmax;
        triangle_aabb(t, &tmin, &tmax);
        array<u32, 3> aligned_min = { static_cast<u32>(progressive_floor(tmin.x)),
                                      static_cast<u32>(progressive_floor(tmin.y)),
                                      static_cast<u32>(progressive_floor(tmin.z)) };
        // if (aligned_min[0] > (grid_size[0] / 2)) continue;

        array<u32, 3> aligned_max = {
            static_cast<u32>(std::floor(tmax.x)),
            static_cast<u32>(std::floor(tmax.y)),
            static_cast<u32>(std::floor(tmax.z)),
        };

        for (u32 x = aligned_min[0]; x <= aligned_max[0]; x++) {
            for (u32 y = aligned_min[1]; y <= aligned_max[1]; y++) {
                for (u32 z = aligned_min[2]; z <= aligned_max[2]; z++) {
                    bool intersecting = true;
                    for (u32 proj_idx = 0; proj_idx < 3 && intersecting; proj_idx++) {
                        vec2 voxel_min = swizzlers[proj_idx](vec3(x, y, z));
                        Triangle proj_triangle;
                        std::transform(t.cbegin(), t.cend(), proj_triangle.begin(), swizzlers[proj_idx]);
                        array<vec2, 4> vertices = { voxel_min,
                                                    voxel_min + 1.0f,
                                                    { voxel_min.x + 1.0f, voxel_min.y },
                                                    { voxel_min.x, voxel_min.y + 1.0f } };

                        for (u32 i = 0; i < 3 && intersecting; i++) {
                            f32 max_signed_dist = -1.0f;
                            for (auto& v : vertices) {
                                f32 dist = signed_edge_function(proj_triangle[i], proj_triangle[(i + 1) % 3],
                                                                back_facing[proj_idx], v);
                                max_signed_dist = std::max(max_signed_dist, dist);
                            }
                            if (max_signed_dist < 0.0f) intersecting = false;
                        }
                    }

                    if (intersecting) {
                        u32 at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                        Voxel::Type ntype = triangle_normal.z >= 0 ? Voxel::CLOSING : Voxel::OPENING;
                        if (grid.at(at).valid && ntype != grid.at(at).type) {
                            // FIXME: could be three triangles in a voxel
                            // the fix: could be using the number of voxel this triangle created on z axis
                            more_than_one_triangle_in_voxel++;
                            grid.at(at).type = Voxel::BOTH;
                        } else if (!grid.at(at).valid) {
                            grid.at(at) = { true, ntype };
                            voxels_n++;
                        }
                    }
                }
            }
        }
    }

    // flood fill
    // x axis is the scanline direction
    if (flood_fill_rast) {
        for (u32 x = 0; x < grid_size[0]; x++) {
            for (u32 y = 0; y < grid_size[1]; y++) {
                Voxel::Type last_type;
                u32 last_z_with_type = std::numeric_limits<u32>::max();
                for (u32 z = 0; z < grid_size[2]; z++) {
                    u32 at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                    if (grid.at(at).valid) {
                        last_type = grid.at(at).type;
                        last_z_with_type = z;
                    } else if (last_type == Voxel::OPENING && z > last_z_with_type) {
                        grid.at(at).valid = true;
                        voxels_n++;
                    }
                }
            }
        }
    }

    if (flood_fill) {
        array<u32, 3> center { grid_size[0] / 2, grid_size[1] / 2, grid_size[2] / 2 };
        flood_fill_rec(grid, grid_size, voxels_n, grid_size[0] / 2, grid_size[1] / 2, grid_size[2] / 2);
        // assert(!grid[0].valid);
        // for (auto& v : grid) v.valid = v.valid ? false : true;
    }

    if (export_magicavoxel) {
        if (export_vox_file("bunny.vox", grid, grid_size, voxels_n)) assert(false && "couldn't not open the vox file");
    } else {
        for (u32 z = 0; z < grid_size[0]; z++)
            for (u32 x = 0; x < grid_size[1]; x++)
                for (u32 y = 0; y < grid_size[2]; y++)
                    // puts(grid[z * grid_size[1] * grid_size[2] + x * grid_size[2] + y] ? "1" : "0");
                    if (grid[z * grid_size[1] * grid_size[2] + x * grid_size[2] + y].valid)
                        printf("%d %d %d 1\n", z, x, y);
                    else
                        printf("%d %d %d 0\n", z, x, y);
    }
    printf("res:\t%u %u %u\n", grid_size[0], grid_size[1], grid_size[2]);
    printf("voxels:\t%u\n", voxels_n);
    return 0;
}
