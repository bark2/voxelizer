#include "iritSkel.h"
#include "obj.h"
#include "types.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

array<Swizzler<vec3>, 3> swizzlers = { get_swizzler<vec3>(0), get_swizzler<vec3>(1),
                                       get_swizzler<vec3>(2) };
bool _gdb = false;

inline f32
signed_edge_function(const vec2& v0, const vec2& v1, const bool back_facing, const vec2& test_point)
{
    f32 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = v1 - v0;
    vec2 edge_normal = d * vec2(-edge.y, edge.x);
    return dot(edge_normal, test_point - v0);
}

inline std::pair<bool, f32>
line_intersection(const vec2& v1, const vec2& v2, const vec2& v3, const vec2& v4)
{
    auto cross = [](const vec2& v1, const vec2 v2) { return v1.x * v2.y - v1.y * v2.x; };
    vec2 a = v3 - v1;
    f32 b = cross(v2 - v1, v4 - v3);
    f32 t = cross(a, v4 - v3) / b;
    f32 u = cross(a, v2 - v1) / b;
    // return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, v1 + t * (v2 - v1) };
    return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, t };
}

inline bool
point_in_triangle(const array<vec2, 3>& proj_triangle, bool back_facing, const vec2& p)
{
    bool result = true;
    const array<array<vec2, 2>, 3> edges = { array<vec2, 2> { proj_triangle[0], proj_triangle[1] },
                                             { proj_triangle[1], proj_triangle[2] },
                                             { proj_triangle[2], proj_triangle[0] } };

    for (auto& e : edges) result &= signed_edge_function(e[0], e[1], back_facing, p) >= 0;

    return result;
}

enum class Intersection_Option { NONE, RETURN_INTERSECTOIN_POINT };

inline bool
triangle_aabb_fconservative_collision(const array<vec2, 3>& proj_triangle,
                                      bool back_facing,
                                      const array<vec2, 2>& aabb)
{
    const array<vec2, 4> aabb_vertices = {
        aabb[0], aabb[1], { aabb[0].x, aabb[1].y }, { aabb[1].x, aabb[0].y }
    };
    for (i32 i = 0; i < 3; i++) {
        f32 max_signed_dist = -1.0f;
        for (auto& v : aabb_vertices) {
            f32 dist =
                signed_edge_function(proj_triangle[i], proj_triangle[(i + 1) % 3], back_facing, v);
            max_signed_dist = std::max(max_signed_dist, dist);
        }
        if (max_signed_dist < 0.0f) return false;
    }
    return true;
}

inline bool
triangle_aabb_conservative_collision(const array<vec2, 3>& proj_triangle,
                                     bool back_facing,
                                     const array<vec2, 2>& aabb)
{
    const vec2 aabb_vertices[4] = {
        aabb[0], aabb[1], { aabb[0].x, aabb[1].y }, { aabb[1].x, aabb[0].y }
    };
    bool proj_voxel_collision = false;
    for (int i = 0; i < 4; i++)
        proj_voxel_collision |= point_in_triangle(proj_triangle, back_facing, aabb_vertices[i]);

    return proj_voxel_collision;
}

inline bool
triangle_aabb_6seperating_collision(const array<vec2, 3>& proj_triangle,
                                    bool back_facing,
                                    const array<vec2, 2>& aabb)
{
    const array<vec2, 4> means = { vec2 { (aabb[0].x + aabb[1].x) / 2, aabb[0].y },
                                   { (aabb[0].x + aabb[1].x) / 2, aabb[1].y },
                                   { aabb[0].x, (aabb[0].y + aabb[1].y) / 2 },
                                   { aabb[1].x, (aabb[0].y + aabb[1].y) / 2 } };
    bool proj_voxel_collision = false;
    for (auto& v : means) proj_voxel_collision |= point_in_triangle(proj_triangle, back_facing, v);
    return proj_voxel_collision;
}

using Intersection_Fun = bool (*)(const array<vec2, 3>&, bool, const array<vec2, 2>&);

inline std::pair<bool, f32>
triangle_voxel_collision(const Triangle& t,
                         array<vec3, 2> voxel,
                         Intersection_Fun intersection_fun,
                         Intersection_Option option)
{
    std::pair<bool, f32> intersecting = { false, 0.0f };
    vec3 triangle_normal = t.normal();
    for (int i = 0; i < 3; i++) {
        Triangle swizzled_triangle = t.gen(swizzlers[i]);
        array<vec2, 3> proj_triangle = { swizzled_triangle[0], swizzled_triangle[1],
                                         swizzled_triangle[2] };
        array<vec2, 2> proj_aabb = { swizzlers[i](voxel[0]), swizzlers[i](voxel[1]) };

        if ((intersecting.first |= intersection_fun(proj_triangle, triangle_normal[i] > 0, proj_aabb)))
            break;
    }

    if (intersecting.first && option == Intersection_Option::RETURN_INTERSECTOIN_POINT) {
        Triangle ztriangle = t.gen(swizzlers[1]);
        const vec2 voxel_vertices[4] = { { voxel[0].z, voxel[0].x },
                                         { voxel[0].z, voxel[1].x },
                                         { voxel[1].z, voxel[1].x },
                                         { voxel[1].z, voxel[0].x } };
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                auto p = line_intersection(ztriangle[i], ztriangle[(i + 1) % 3], voxel_vertices[j],
                                           voxel_vertices[(j + 1) % 4]);
                if (p.first) intersecting.second = std::max(intersecting.second, p.second);
            }
        }
    }

    return intersecting;
}

inline bool
aabb_collision(const vec3& b1_min, const vec3& b1_max, const vec3& b2_min, const vec3& b2_max)
{
    bool result = false;
    if ((b2_max.x - b1_min.x) * (b2_min.x - b1_max.x) >= 0.0 ||
        (b2_max.y - b1_min.y) * (b2_min.y - b1_max.y) >= 0.0 ||
        (b2_max.z - b1_min.z) * (b2_min.z - b1_max.z) >= 0.0)
        result = true;
    return result;
}

inline i32
dominant_axis(const vec3& tnormal, array<vec3, 3> axises)
{
    array<f32, 3> normal_projections = { std::abs(dot(axises[0], tnormal)),
                                         std::abs(dot(axises[1], tnormal)),
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
    return std::max(static_cast<f32>(0.0f), f == std::floor(f) ? f - 1 : std::floor(f));
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

// TODO: read IRIT data
// TODO: check the bloodfill

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
    bool do_flood_fill_inv;
    if (get_cmd(argv, argv + argc, "--flood-fill-inv")) do_flood_fill_inv = true;

    bool do_export_magicavoxel;
    char** arg_format = get_cmd(argv, argv + argc, "--magicavoxel");
    if (arg_format) do_export_magicavoxel = true;

    // std::vector<bool> grid(grid_size[0] * grid_size[1] * grid_size[2], false);
    std::vector<Voxel> grid(grid_size[0] * grid_size[1] * grid_size[2],
                            { false, Voxel::NONE, 0.0f });

    extern vec3 scene_aabb_min;
    extern vec3 scene_aabb_max;
    extern std::vector<Triangle> triangles;
    if (false) {
        // const char* file_name = "data/BasicModels/cube.itd";
        const char* file_name = "data/cow.itd";
        CGSkelProcessIritDataFiles((const char* const*)&file_name, 1);
        for (auto& t : triangles) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    scene_aabb_max[j] = std::max(scene_aabb_max[i], t[i][j]);
                    scene_aabb_min[j] = std::min(scene_aabb_min[i], t[i][j]);
                }
            }
        }
    } else {
        // const auto meshes = ai_load_obj_file("data/lowpolydeer/deer.obj");
        const auto meshes = ai_load_obj_file("data/bunny.obj");
        // const auto meshes = ai_load_obj_file("data/kitten.obj");
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

    f32 scene_aabb_min_min = std::min({ scene_aabb_min[0], scene_aabb_min[1], scene_aabb_min[2] });
    f32 scene_aabb_max_max = std::max({ scene_aabb_max[0], scene_aabb_max[1], scene_aabb_max[2] });
    vec3 voxel_size = { (scene_aabb_max[0] - scene_aabb_min[0]) / grid_size[0],
                        (scene_aabb_max[1] - scene_aabb_min[1]) / grid_size[1],
                        (scene_aabb_max[2] - scene_aabb_min[2]) / grid_size[2] };

    // FIXME: slow!
    for (auto& t : triangles)
        for (auto& v : t)
            for (i32 i = 0; i < 3; i++)
                v[i] = (grid_size[i] - 1.0f) * (v[i] - scene_aabb_min[i]) /
                       (scene_aabb_max_max - scene_aabb_min_min);

    u32 voxels_n = 0;
    array<u32, 3> mesh_center {};
    for (auto& t : triangles) {
        vec3 triangle_normal = t.normal();
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

                    auto intersecting =
                        triangle_voxel_collision(t, aabb, triangle_aabb_fconservative_collision, option);

                    if (intersecting.first && option == Intersection_Option::RETURN_INTERSECTOIN_POINT) {
                        // if (x == 0 && y == 21 && z == 25) std::raise(SIGINT);
                        auto& voxel = grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z);
                        if (intersecting.second < voxel.max_intersection_off) continue;
                        assert(intersecting.second >= 0.0f && intersecting.second <= 1.0f);

                        Voxel::Type type;
                        // if (std::abs(triangle_normal.z) <= std::numeric_limits<f32>::epsilon())
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

    // flood fill
    // array<i32, 3> threshold = { 0, grid_size[1] / 2, 0 };
    array<i32, 3> threshold = { 0, 0, 0 };

    // x axis is the scanline direction
    if (do_flood_fill_rast)
        flood_fill_rast(grid, grid_size, voxels_n);
    else if (do_flood_fill_rec)
        flood_fill_rec(grid, grid_size, voxels_n, mesh_center[0], mesh_center[1], mesh_center[2]);
    else if (do_flood_fill_inv)
        flood_fill_inv(grid, grid_size, voxels_n);

    if (do_export_magicavoxel) {
        if (export_magicavoxel("bunny.vox", grid, grid_size, voxels_n))
            assert(0 && "couldn't not open the vox file");
    } else {
        std::vector<u8> bool_grid(grid_size[0] * grid_size[1] * grid_size[2], '0');
        std::transform(grid.begin(), grid.end(), bool_grid.begin(),
                       [](const Voxel& v) { return v.valid ? '1' : '0'; });
        FILE* out = fopen("bunny.bin", "wb");
        if (!out) return 1;
        if (fwrite(bool_grid.data(), sizeof(u8), bool_grid.size(), out) != bool_grid.size()) return 1;
        fclose(out);
    }
    printf("res:\t%u %u %u\n", grid_size[0], grid_size[1], grid_size[2]);
    printf("voxels:\t%u\n", voxels_n);
    printf("mesh center:\t%u %u %u\n", mesh_center[0], mesh_center[1], mesh_center[2]);
    return 0;
}
