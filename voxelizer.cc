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
    const vec3 critical
        = { tnormal.x > 0.0f ? vdelta : 0.0f, tnormal.y > 0.0f ? vdelta : 0.0f,
            tnormal.z > 0.0f ? vdelta : 0.0f };
    const f32 d[2] = {
        glm::dot(tnormal, critical - t[0]),
        glm::dot(tnormal, (vec3(vdelta, vdelta, vdelta) - critical) - t[0]),
    };
    const f32 back_facing[3]
        = { tnormal.z > 0 ? 1.0f : -1.0f, tnormal.y > 0 ? 1.0f : -1.0f,
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
            offsets[i][j] = -glm::dot(enormals[i][j], { t[j].x, t[j].y })
                + std::max(0.0f, vdelta * enormals[i][j].x)
                + std::max(0.0f, vdelta * enormals[i][j].y);

    Setup_PEF result;
    result.tedges = tedges;
    result.tnormal = tnormal;
    std::memcpy(result.enormals, enormals, sizeof(enormals));
    std::memcpy(result.d, d, sizeof(d));
    std::memcpy(result.offsets, offsets, sizeof(offsets));
    // Do we need to move?
    return std::move(result);
}

bool
collision_pef(const Setup_PEF& s, const vec3 vmin)
{
    bool result = false;
    bool planes_intersecting
        = (glm::dot(s.tnormal, vmin) + s.d[0]) * (glm::dot(s.tnormal, vmin) + s.d[1])
        <= 0;
    if (planes_intersecting) return true;
    bool positive_signed_edge_distance;
    for (u32 i = 0; i < 3; i++)
        for (u32 j = 0; j < 3; j++)
            positive_signed_edge_distance
                &= glm::dot(s.enormals[i][j], vec2(vmin[i], vmin[i != 2 ? i + 1 : 0]))
                    + s.offsets[i][j]
                >= 0;
    if (planes_intersecting & positive_signed_edge_distance) result = true;
    return result;
}

inline bool
signed_edge_function(
    const array<vec2, 2>& edge_vertices, bool back_facing, const vec2& test_point)
{
    f32 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = edge_vertices[1] - edge_vertices[0];
    const vec2 edge_normal = d * vec2(-edge.y, edge.x);
    auto result = glm::dot(edge_normal, test_point - edge_vertices[0]) >= 0;
    // printf("%s", result? "signed_edge_function!\n" : "");
    return result;
}

inline bool
triangle_point_collision(
    const array<vec2, 3>& projected_triangle, bool back_facing, const vec2& p)
{
    // XY, YZ, ZX
    auto result = signed_edge_function(
                      { projected_triangle[0], projected_triangle[1] }, back_facing, p)
        && signed_edge_function(
                      { projected_triangle[1], projected_triangle[2] }, back_facing, p)
        && signed_edge_function(
                      { projected_triangle[2], projected_triangle[0] }, back_facing, p);
    // printf("%s", result? "triangle_point_collision!\n" : "");
    return result;
}

inline bool
triangle_aabb_conservative_collision(
    const array<vec2, 3>& projected_triangle,
    bool back_facing,
    const array<vec2, 2>& aabb)
{
    array<vec2, 4> aabb_vertices
        = { aabb[0], aabb[1], { aabb[0].x, aabb[1].y }, { aabb[1].x, aabb[0].y } };
    for (auto& v : aabb_vertices) {
        if (triangle_point_collision(projected_triangle, back_facing, v)) {
            // printf("triangle_conservative_collision!\n");
            return true;
        }
    }
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

using Projection = vec3 (*)(const vec3&);
array<Projection, 3> projections = { to_xy, to_yz, to_zx };

inline bool
triangle_voxel_collision(
    const array<vec2, 3>& projected_triangle,
    bool back_facing,
    const vec3& vmin,
    const vec3& vmax,
    u32 projection_axis)
{
    array<vec3, 2> voxel = { vmin, vmax };
    array<vec2, 2> projected_voxel
        = { projections[projection_axis](vmin), projections[projection_axis](vmax) };
    auto result = triangle_aabb_conservative_collision(
        projected_triangle, back_facing, projected_voxel);
    // printf("%s", cresult ? "triangle_voxel_collision!\n" : "");
    return result;
}

inline bool
aabb_collision(
    const vec3& b1_min, const vec3& b1_max, const vec3& b2_min, const vec3& b2_max)
{
    bool result = false;
    if ((b2_max.x - b1_min.x) * (b2_min.x - b1_max.x) >= 0.0
        || (b2_max.y - b1_min.y) * (b2_min.y - b1_max.y) >= 0.0
        || (b2_max.z - b1_min.z) * (b2_min.z - b1_max.z) >= 0.0)
        result = true;
    return result;
}

inline u32
dominant_axis(vec3 tnormal, array<vec3, 3> axises)
{
    array<f32, 3> normal_projections
        = { glm::dot(axises[0], tnormal), glm::dot(axises[1], tnormal),
            glm::dot(axises[2], tnormal) };
    return std::max_element(normal_projections.cbegin(), normal_projections.cend())
        - normal_projections.cbegin();
}

inline std::pair<vec3, vec3>
triangle_aabb(const Triangle& t)
{
    vec3 min
        = { std::min({ t[0].x, t[1].x, t[2].x }), std::min({ t[0].y, t[1].y, t[2].y }),
            std::min({ t[0].z, t[1].z, t[2].z }) };
    vec3 max
        = { std::max({ t[0].x, t[1].x, t[2].x }), std::max({ t[0].y, t[1].y, t[2].y }),
            std::max({ t[0].z, t[1].z, t[2].z }) };
    return std::make_pair(min, max);
}

int
main(int argv, char* args[])
{
    // const u32 resulotion = std::strtoul(args[1], nullptr, 10);
    const u32 resulotion = std::pow(2, 6);
    const f32 fresolution = static_cast<f32>(resulotion);
    // const f32 voxel_size = 1 / static_cast<f32>(resulotion);
    const f32 voxel_size = 1.0f;
    std::vector<bool> grid(resulotion * resulotion * resulotion, false);

    // normalized to range [0, 1], easily portable to a scene
    auto meshes = load_obj_file("data/bunny.obj");
    auto mesh = meshes[0];

    std::vector<Triangle> triangles;
    triangles.reserve(mesh.vertices.size() / 3);
    for (u32 i = 0; i < mesh.indices.size(); i += 3) {
        u32 cindices[] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };
        Triangle t;
        t[0] = mesh.vertices[cindices[0]].pos * fresolution;
        t[1] = mesh.vertices[cindices[1]].pos * fresolution;
        t[2] = mesh.vertices[cindices[2]].pos * fresolution;
        triangles.emplace_back(t);
    }

    const array<vec3, 3> axises
        = { vec3(0.0f, 0.0f, 1.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f) };

    u32 voxels_n = 0;
    for (auto& t : triangles) {
        // auto setup = setup_pef(voxel_size, t);
        vec3 triangle_normal = t.normal();
        u32 projection_axis = dominant_axis(triangle_normal, axises);
        bool back_facing = triangle_normal[projection_axis] > 0;
        Triangle rotated_triangle;
        std::transform(
            t.cbegin(), t.cend(), rotated_triangle.begin(), projections[projection_axis]);
        const array<vec2, 3>& projected_triangle
            = { rotated_triangle[0], rotated_triangle[1], rotated_triangle[2] };
        auto [tmin, tmax] = triangle_aabb(rotated_triangle);
        f32 dzdx = (tmax.z - tmin.z) / (tmax.x - tmin.x);
        f32 dzdy = (tmax.z - tmin.z) / (tmax.y - tmin.y);

        vec3 aabb_min = { std::floor(tmin.x), std::floor(tmin.y), std::floor(tmin.z) };
        vec3 aabb_max = { std::ceil(tmax.x), std::ceil(tmax.y), std::ceil(tmax.z) };
        u32 ctriangle_tests = 0, ctriangle_collisions = 0;
        // triangle's axis-aligned bounding box voxels
        for (f32 x = aabb_min.x, tz = tmin.z; x <= aabb_max.x - voxel_size;
             x += voxel_size, tz += dzdx) {
            for (f32 y = aabb_min.y; y <= aabb_max.y - voxel_size;
                 y += voxel_size, y += dzdy) {
                for (auto&& z :
                     { std::floor(tz), std::ceil(tz), std::floor(tz + 1.0f) }) {
                    ctriangle_tests++;
                    vec3 voxel_min = { x, y, z };
                    // printf("\t%s\n", glm::to_string(voxel_min).c_str());
                    bool intersecting = triangle_voxel_collision(
                        projected_triangle, back_facing, voxel_min,
                        { voxel_min + voxel_size }, projection_axis);
                    if (intersecting) {
                        ctriangle_collisions++;
                        u32 ux = static_cast<u32>(x);
                        u32 uy = static_cast<u32>(y);
                        u32 uz = static_cast<u32>(z);
                        u32 at = ux * resulotion * resulotion + uy * resulotion + uz;
                        if (!grid.at(at)) { voxels_n++; }
                        grid.at(at) = true;
                    }
                }
            }
        }
    }

    int err = export_vox_file("bunny.vox", grid, resulotion, voxels_n);
    assert(!err);
    printf("resulotion: %f\n", static_cast<f32>(resulotion));
    printf("voxels_n: %u\n", voxels_n);
    // for (u32 x = 0; x < resulotion; x++)
    // for (u32 y = 0; y < resulotion; y++)
    // for (u32 z = 0; z < resulotion; z++)
    // if (grid[x * resulotion * resulotion + y * resulotion + z])
    // printf("%u %u %u\n", x, y, z);
    return 0;
}
