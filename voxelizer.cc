#include "obj.h"
#include "types.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

inline f32
signed_edge_function(const array<vec2, 2>& edge_vertices,
                     const bool back_facing,
                     const vec2& test_point)
{
    f32 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = edge_vertices[1] - edge_vertices[0];
    vec2 edge_normal = d * vec2(-edge.y, edge.x);
    return dot(edge_normal, test_point - edge_vertices[0]);
}

inline bool
point_in_triangle(const array<vec2, 3>& projected_triangle, bool back_facing, const vec2& p)
{
    bool result = true;
    const array<array<vec2, 2>, 3> edges = {
        array<vec2, 2> { projected_triangle[0], projected_triangle[1] },
        { projected_triangle[1], projected_triangle[2] },
        { projected_triangle[2], projected_triangle[0] }
    };

    for (auto& e : edges) result &= signed_edge_function(e, back_facing, p) >= 0;

    return result;
}

inline bool
triangle_aabb_conservative_collision(const array<vec2, 3>& projected_triangle,
                                     bool back_facing,
                                     const array<vec2, 2>& aabb)
{
    const array<vec2, 4> aabb_vertices = {
        aabb[0], aabb[1], { aabb[0].x, aabb[1].y }, { aabb[1].x, aabb[0].y }
    };
    for (auto& v : aabb_vertices)
        if (point_in_triangle(projected_triangle, back_facing, v)) return true;

    return false;
}

inline bool
triangle_aabb_6seperating_collision(const array<vec2, 3>& projected_triangle,
                                    bool back_facing,
                                    const array<vec2, 2>& aabb)
{
    const array<vec2, 4> means = { vec2 { (aabb[0].x + aabb[1].x) / 2, aabb[0].y },
                                   { (aabb[0].x + aabb[1].x) / 2, aabb[1].y },
                                   { aabb[0].x, (aabb[0].y + aabb[1].y) / 2 },
                                   { aabb[1].x, (aabb[0].y + aabb[1].y) / 2 } };
    for (auto& v : means)
        if (point_in_triangle(projected_triangle, back_facing, v)) return true;

    return false;
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

inline u32
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

inline u32
progressive_floor(f32 f)
{
    return std::max(0.0f, f == std::floor(f) ? f - 1 : std::floor(f));
}
inline u32
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
    if (itr != end && ++itr != end) { return itr; }
    return nullptr;
}

// TODO: read IRIT data
// (n*m*k) zeros/ones, one per line in lexicographic order (k first)
// bloodfill

int
main(int argc, char* argv[])
{
    bool export_magicavoxel;
    array<u32, 3> grid_size;
    char** resolution = get_cmd(argv, argv + argc, "-d");
    if (resolution)
        sscanf(*resolution, "%u,%u,%u", &grid_size[0], &grid_size[1], &grid_size[2]);
    else
        assert(0 && "wrong grid resolution input");

    char** output_format = get_cmd(argv, argv + argc, "-v");
    if (get_cmd(argv, argv + argc, "-v")) export_magicavoxel = true;

    std::vector<bool> grid(grid_size[0] * grid_size[1] * grid_size[2], false);

    // normalized to range [0, 1], easily portable to a scene
    const auto meshes = load_obj_file("data/bunny.obj");
    const auto& mesh = meshes[0];

    std::vector<Triangle> triangles;
    std::vector<vec3> normals;
    triangles.reserve(mesh.vertices.size() / 3);
    for (u32 i = 0; i < mesh.indices.size(); i += 3) {
        Triangle t;
        for (u32 j = 0; j < 3; j++) {
            t[0][j] = mesh.vertices[mesh.indices[i]].pos[j] * grid_size[j];
            t[1][j] = mesh.vertices[mesh.indices[i + 1]].pos[j] * grid_size[j];
            t[2][j] = mesh.vertices[mesh.indices[i + 2]].pos[j] * grid_size[j];
        }
        triangles.emplace_back(t);
    }

    // Has to be sorted : X, Y, Z
    const array<vec3, 3> axises = {
        vec3 { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };

    u32 voxels_n = 0;
    u32 max_zrange = 0;
    for (auto& t : triangles) {
        vec3 triangle_normal = t.normal();
        u32 projection_axis = dominant_axis(triangle_normal, axises);
        bool back_facing = triangle_normal[projection_axis] > 0;

        auto inv_swizzler = get_inv_swizzler<array<u32, 3>>(projection_axis);
        Swizzler<array<u32, 3>> array_swizzler = get_swizzler<array<u32, 3>>(projection_axis);
        Triangle projected_triangle;
        std::transform(t.cbegin(), t.cend(), projected_triangle.begin(),
                       get_swizzler<vec3>(projection_axis));

        vec3 tmin, tmax;
        triangle_aabb(projected_triangle, &tmin, &tmax);
        array<u32, 3> voxel_min_min = { progressive_floor(tmin.x), progressive_floor(tmin.y),
                                        progressive_floor(tmin.z) };
        array<u32, 3> projected_grid_size = array_swizzler(grid_size);
        array<u32, 3> voxel_min_max = {
            std::min(projected_grid_size[0] - 1, static_cast<u32>(std::floor(tmax.x))),
            std::min(projected_grid_size[1] - 1, static_cast<u32>(std::floor(tmax.y))),
            std::min(projected_grid_size[2] - 1, static_cast<u32>(std::floor(tmax.z))),
        };

        for (u32 x = voxel_min_min[0]; x <= voxel_min_max[0]; x++) {
            for (u32 y = voxel_min_min[1]; y <= voxel_min_max[1]; y++) {
                u32 czrange = 0;
                for (u32 z = voxel_min_min[2]; z <= voxel_min_max[2]; z++) {
                    vec2 voxel_min = { static_cast<f32>(x), static_cast<f32>(y) };
                    vec2 voxel_max = voxel_min + 1.0f;
                    if (triangle_aabb_conservative_collision(to_vec2_array(projected_triangle),
                                                             back_facing,
                                                             { voxel_min, voxel_max })) {
                        czrange++;
                        // if ( czrange > 3) std::raise(SIGINT);
                        array<u32, 3> original_voxel_min = inv_swizzler({ x, y, z });
                        assert(original_voxel_min[0] < grid_size[0] &&
                               original_voxel_min[1] < grid_size[1] &&
                               original_voxel_min[2] < grid_size[2] &&
                               "assigning to grid out of bound");
                        u32 at = original_voxel_min[0] * grid_size[1] * grid_size[2] +
                                 original_voxel_min[1] * grid_size[2] + original_voxel_min[2];
                        if (!grid[at]) voxels_n++;
                        grid[at] = true;
                    }
                }
                max_zrange = std::max(max_zrange, czrange);
            }
        }
    }

    if (export_magicavoxel) {
        if (export_vox_file("bunny.vox", grid, grid_size, voxels_n))
            assert(false && "couldnt not openg the vox file");
    } else {
        for (u8 x = 0; x < grid_size[0]; x++)
            for (u8 y = 0; y < grid_size[1]; y++)
                for (u8 z = 0; z < grid_size[2]; z++)
                    ;
    }
    printf("res:\t%u %u %u\n", grid_size[0], grid_size[1], grid_size[2]);
    printf("voxels:\t%u\n", voxels_n);
    printf("zrange: %u\n", max_zrange);

    return 0;
}
