#include "obj.h"
#include "types.h"
#include <algorithm>
#include <utility>

struct Setup_PEF {
    std::array<vec3, 3> tedges;
    vec3 tnormal;
    vec2 enormals[3][3];
    f32 d[2];
    f32 offsets[3][3];
};

Setup_PEF
setup_pef(f32 vdelta, const Triangle& t)
{
    const array<vec3, 3> tedges = { t[1] - t[0], t[2] - t[1], t[0] - t[2] };
    const vec3 tnormal = glm::cross(tedges[1], tedges[0]);
    const vec3 critical = { tnormal.x > 0.0f ? vdelta : 0.0f, tnormal.y > 0.0f ? vdelta : 0.0f,
                            tnormal.z > 0.0f ? vdelta : 0.0f };
    const f32 d[2] = {
        glm::dot(tnormal, critical - t[0]),
        glm::dot(tnormal, (vec3(vdelta, vdelta, vdelta) - critical) - t[0]),
    };
    const f32 back_facing[3] = { tnormal.z > 0 ? 1.0f : -1.0f, tnormal.y > 0 ? 1.0f : -1.0f,
                                 tnormal.x > 0 ? 1.0f : -1.0f };
    // XY, YZ, ZX
    const vec2 enormals[3][3] = { { back_facing[0] * vec2(-tedges[0].y, tedges[0].x),
                                    back_facing[0] * vec2(-tedges[1].y, tedges[1].x),
                                    back_facing[0] * vec2(-tedges[2].y, tedges[2].x) },
                                  { back_facing[1] * vec2(-tedges[0].z, tedges[0].y),
                                    back_facing[1] * vec2(-tedges[1].z, tedges[1].y),
                                    back_facing[1] * vec2(-tedges[2].z, tedges[2].y) },
                                  { back_facing[2] * vec2(-tedges[0].x, tedges[0].z),
                                    back_facing[2] * vec2(-tedges[1].x, tedges[1].z),
                                    back_facing[2] * vec2(-tedges[2].x, tedges[2].z) } };
    f32 offsets[3][3];
    for (u32 i = 0; i < 3; i++)
        for (u32 j = 0; j < 3; j++)
            offsets[i][j] = -glm::dot(enormals[i][j], { t[j].x, t[j].y }) +
                std::max(0.0f, vdelta * enormals[i][j].x) +
                std::max(0.0f, vdelta * enormals[i][j].y);

    Setup_PEF result;
    result.tedges = tedges;
    result.tnormal = tnormal;
    std::memcpy(result.enormals, enormals, sizeof(enormals));
    std::memcpy(result.d, d, sizeof(d));
    std::memcpy(result.offsets, offsets, sizeof(offsets));
    return result;
}

bool
collision_pef(const Setup_PEF& s, const vec3 vmin)
{
    bool result = false;
    bool planes_intersecting =
        (glm::dot(s.tnormal, vmin) + s.d[0]) * (glm::dot(s.tnormal, vmin) + s.d[1]) <= 0;
    if (planes_intersecting) return true;
    bool positive_signed_edge_distance;
    for (u32 i = 0; i < 3; i++)
        for (u32 j = 0; j < 3; j++)
            positive_signed_edge_distance &=
                glm::dot(s.enormals[i][j], vec2(vmin[i], vmin[i != 2 ? i + 1 : 0])) +
                    s.offsets[i][j] >=
                0;
    if (planes_intersecting & positive_signed_edge_distance) result = true;
    return result;
}

inline f32
signed_edge_function(
    const array<vec2, 2>& edge_vertices, bool back_facing, const vec2& test_point)
{
    f32 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = edge_vertices[1] - edge_vertices[0];
    vec2 edge_normal = d * vec2(-edge.y, edge.x);
    return glm::dot(edge_normal, test_point - edge_vertices[0]);
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
triangle_aabb_conservative_collision(
    const array<vec2, 3>& projected_triangle, bool back_facing, const array<vec2, 2>& aabb)
{
    const array<vec2, 4> aabb_vertices = {
        aabb[0], aabb[1], { aabb[0].x, aabb[1].y }, { aabb[1].x, aabb[0].y }
    };
    for (auto& v : aabb_vertices)
        if (point_in_triangle(projected_triangle, back_facing, v)) return true;

    return false;
}

inline bool
triangle_aabb_6seperating_collision(
    const array<vec2, 3>& projected_triangle, bool back_facing, const array<vec2, 2>& aabb)
{
    const array<vec2, 4> means = { vec2 { (aabb[0].x + aabb[1].x) / 2, aabb[0].y },
                                   { (aabb[0].x + aabb[1].x) / 2, aabb[1].y },
                                   { aabb[0].x, (aabb[0].y + aabb[1].y) / 2 },
                                   { aabb[1].x, (aabb[0].y + aabb[1].y) / 2 } };
    for (auto& v : means)
        if (point_in_triangle(projected_triangle, back_facing, v)) return true;

    return false;
}

inline vec3
to_xy(const vec3& v)
{
    return { v.x, v.y, v.z };
};
inline vec3
to_yz(const vec3& v)
{
    return { v.y, v.z, v.x };
};
inline vec3
to_zx(const vec3& v)
{
    return { v.z, v.x, v.y };
};
inline array<u32, 3>
inversed_xy(const array<u32, 3>& v)
{
    return { v[0], v[1], v[2] };
};
inline array<u32, 3>
inversed_yz(const array<u32, 3>& v)
{
    return { v[2], v[0], v[1] };
};
inline array<u32, 3>
inversed_zx(const array<u32, 3>& v)
{
    return { v[1], v[2], v[0] };
};

using Projection = vec3 (*)(const vec3&);
using Inv_Projection = array<u32, 3> (*)(const array<u32, 3>&);
array<Projection, 3> projections = { to_yz, to_zx, to_xy };
array<Inv_Projection, 3> inverse_projections = { inversed_yz, inversed_zx, inversed_xy };

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
    array<f32, 3> normal_projections = { glm::abs(glm::dot(axises[0], tnormal)),
                                         glm::abs(glm::dot(axises[1], tnormal)),
                                         glm::abs(glm::dot(axises[2], tnormal)) };
    auto max_index = std::max_element(normal_projections.cbegin(), normal_projections.cend());
    return max_index - normal_projections.cbegin();
}

inline std::pair<vec3, vec3>
triangle_aabb(const Triangle& t)
{
    vec3 min = { std::min({ t[0].x, t[1].x, t[2].x }), std::min({ t[0].y, t[1].y, t[2].y }),
                 std::min({ t[0].z, t[1].z, t[2].z }) };
    vec3 max = { std::max({ t[0].x, t[1].x, t[2].x }), std::max({ t[0].y, t[1].y, t[2].y }),
                 std::max({ t[0].z, t[1].z, t[2].z }) };
    return std::make_pair(min, max);
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

int
main(int argv, char* args[])
{
    const u32 resulotion = std::pow(2, 7);
    const f32 fresulotion = static_cast<f32>(resulotion);
    const f32 voxel_size = 1.0f;
    std::vector<bool> grid(resulotion * resulotion * resulotion, false);

    // normalized to range [0, 1], easily portable to a scene
    const auto meshes = load_obj_file_("data/bunny.obj");
    const auto& mesh = meshes[0];

    std::vector<Triangle> triangles;
    std::vector<vec3> normals;
    triangles.reserve(mesh.vertices.size() / 3);
    for (u32 i = 0; i < mesh.indices.size(); i += 3) {
        u32 cindices[] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
        Triangle t;
        t[0] = mesh.vertices[cindices[0]].pos * (fresulotion - 1.0f);
        t[1] = mesh.vertices[cindices[1]].pos * (fresulotion - 1.0f);
        t[2] = mesh.vertices[cindices[2]].pos * (fresulotion - 1.0f);
        triangles.emplace_back(t);
    }

    // Has to be sorted : X, Y, Z
    const array<vec3, 3> axises = {
        vec3 { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f },
    };

    u32 voxels_n = 0;
    f32 max_zrange = 0;
    for (auto& t : triangles) {
        // auto setup = setup_pef(voxel_size, t);
        vec3 triangle_normal = t.normal();
        u32 projection_axis = dominant_axis(triangle_normal, axises);
        auto inverse_projection = inverse_projections[projection_axis];
        bool back_facing = triangle_normal[projection_axis] > 0;
        Triangle projected_triangle;
        std::transform(
            t.cbegin(), t.cend(), projected_triangle.begin(), projections[projection_axis]);

        auto [tmin, tmax] = triangle_aabb(projected_triangle);
        array<u32, 3> aabb_min = { progressive_floor(tmin.x), progressive_floor(tmin.y),
                                   progressive_floor(tmin.z) };
        // array<u32, 3> aabb_max = { progressive_ceil(tmax.x, resulotion - 1),
        //                            progressive_ceil(tmax.y, resulotion - 1),
        //                            progressive_ceil(tmax.z, resulotion - 1) };
        // std::floor because the triple for loop is on the voxel_min?
        array<u32, 3> aabb_max = { static_cast<u32>(std::floor(tmax.x)),
                                   static_cast<u32>(std::floor(tmax.y)),
                                   static_cast<u32>(std::floor(tmax.z)) };

        // max_zrange = std::max(max_zrange, aabb_max[2] - aabb_min.z + 1);
        f32 tz = tmin.z;
        for (u32 x = aabb_min[0]; x <= aabb_max[0]; x++) {
            for (u32 y = aabb_min[1]; y <= aabb_max[1]; y++) {
                for (u32 z = aabb_min[2]; z <= aabb_max[2]; z++) {
                    vec2 voxel_min = { x, y };
                    vec2 voxel_max = voxel_min + voxel_size;
                    bool intersecting = triangle_aabb_conservative_collision(
                        to_vec2_array(projected_triangle), back_facing,
                        { voxel_min, voxel_max });
                    if (intersecting) {
                        auto camera_space_voxel = inverse_projection({ x, y, z });
                        assert(
                            camera_space_voxel[0] < resulotion &&
                            camera_space_voxel[1] < resulotion &&
                            camera_space_voxel[2] < resulotion);
                        u32 at = camera_space_voxel[0] * resulotion * resulotion +
                            camera_space_voxel[1] * resulotion + camera_space_voxel[2];
                        if (!grid.at(at)) voxels_n++;
                        grid.at(at) = true;
                    }
                }
            }
        }
    }

    if (export_vox_file("bunny.vox", grid, resulotion, voxels_n)) assert(0);
    printf("res:\t%u\n", resulotion);
    printf("voxels:\t%u\n", voxels_n);
    printf("zrange: %f\n", max_zrange);
    return 0;
}
