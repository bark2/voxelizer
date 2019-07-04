#pragma once

#include "types.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <tuple>
#include <utility>

template <typename Vec>
Vec
swizzle(Vec v, int n = 1)
{
    if (!n) return v;

    for (typename Vec::size_type i = 0; i < v.size() - 1; i++) std::swap(v[i], v[(i + 1) % v.size()]);
    return swizzle(v, n - 1);
}

inline f32
signed_edge_function(const vec2& v0, const vec2& v1, const bool back_facing, const vec2& test_point)
{
    f32 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = v1 - v0;
    vec2 edge_normal = d * vec2(-edge.y, edge.x);
    return dot(edge_normal, test_point - v0);
}

inline f32
signed_edge_function(const array<vec2, 2>& e1, const bool back_facing, const vec2& test_point)
{
    return signed_edge_function(e1[0], e1[1], back_facing, test_point);
}

inline f32 signed_plane_function(array<vec3, 3> plane, const vec2& test_point);

inline std::pair<bool, vec2>
line_intersection(const vec2& v1, const vec2& v2, const vec2& v3, const vec2& v4)
{
    auto cross = [](const vec2& v1, const vec2 v2) { return v1.x * v2.y - v1.y * v2.x; };
    vec2 a = v3 - v1;
    f32 b = cross(v2 - v1, v4 - v3);
    f32 t = cross(a, v4 - v3) / b;
    f32 u = cross(a, v2 - v1) / b;
    vec2 intersection_point = v1 + t * (v2 - v1);
    // if (return_info) *return_info = (t == 0.0f || t == 1.0f || u == 0.0f || u == 1.0f);
    // return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, intersection_point };
    return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, vec2(t, u) };
}

inline std::pair<bool, vec2>
line_intersection(const array<vec2, 2>& e1, const array<vec2, 2>& e2)
{
    return line_intersection(e1[0], e1[1], e2[0], e2[1]);
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

inline std::pair<bool, f32>
line_plane_intersection(const array<vec3, 3>& plane, const array<vec3, 2>& line)
{
    vec3 normal = cross(plane[1] - plane[0], plane[2] - plane[0]);
    f32 denom = dot(normal, line[1] - line[0]);
    if (!denom) return { true, -1.0f };

    float t = dot(plane[0] - line[0], normal) / denom;
    if (t >= 0.0f && t <= 1.0f) return { true, t };
    return { false, {} };
}

array<f32, 3>
get_barycentric(const Triangle& t, const vec3& p)
{
    vec3 v0 = t[1] - t[0], v1 = t[2] - t[0], v2 = p - t[0];
    f32 d00 = dot(v0, v0);
    f32 d01 = dot(v0, v1);
    f32 d11 = dot(v1, v1);
    f32 d20 = dot(v2, v0);
    f32 d21 = dot(v2, v1);
    f32 denom = d00 * d11 - d01 * d01;
    f32 v = (d11 * d20 - d01 * d21) / denom;
    f32 w = (d00 * d21 - d01 * d20) / denom;
    f32 u = 1.0f - v - w;
    return { v, w, u };
}

inline std::pair<bool, vec3> line_intersection(const array<vec3, 2>& e1, const array<vec3, 2>& e2);

inline std::pair<bool, vec3>
line_triangle_intersection(const Triangle& t, const array<vec3, 2>& line)
{
    auto plane_intersection = line_plane_intersection(t, line);
    if (!plane_intersection.first) return { false, {} };
    if (plane_intersection.second < 0.0f) {
        /* the line lies on t's plane
         * both or only one point could be in t
         * if both of them in t: return the max
         * else return the intersectiton to all t's edges.
         * else there is no intersection thus return false     */
        array<f32, 3> barycentrics_l0 = get_barycentric(t, line[0]);
        array<f32, 3> barycentrics_l1 = get_barycentric(t, line[1]);
        bool is_l0_in_triangle = std::all_of(barycentrics_l0.cbegin(), barycentrics_l0.cend(),
                                             [](f32 x) { return (x >= 0.0f && x <= 1.0f); });
        bool is_l1_in_triangle = std::all_of(barycentrics_l1.cbegin(), barycentrics_l1.cend(),
                                             [](f32 x) { return (x >= 0.0f && x <= 1.0f); });
        if (!is_l0_in_triangle && !is_l1_in_triangle) {
            // assert(0); // FIXME
            return { false, {} };
        } else if (is_l0_in_triangle != is_l1_in_triangle) {
            vec3 v = -line[0] / (line[1] - line[0]);
            for (auto& x : v)
                if (x >= 0.0f && x <= 1.0f) return { true, line[0] + v * (line[1] - line[0]) };
            // assert(0); // FIXME
            return { false, {} };
        } else {
            auto max_point = line[0].z > line[1].z ? line[0] : line[1];
            return { false, max_point };
        }
    }
    vec3 p = line[0] + plane_intersection.second * (line[1] - line[0]);
    array<f32, 3> barycentrics = get_barycentric(t, p);
    bool is_point_in_triangle = std::all_of(barycentrics.cbegin(), barycentrics.cend(),
                                            [](f32 x) { return (x >= 0.0f && x <= 1.0f); });
    return { is_point_in_triangle, p };
}

enum class Intersection_Option { NONE, RETURN_INTERSECTOIN_POINT };

inline bool
has_seperating_plane(const array<vec2, 3>& proj_triangle, bool back_facing, const array<vec2, 2>& square)
{
    const array<vec2, 4> square_vertices = {
        square[0], square[1], { square[0].x, square[1].y }, { square[1].x, square[0].y }
    };

    for (i32 i = 0; i < 3; i++) {
        f32 max_signed_dist = -std::numeric_limits<f32>::max();
        for (auto& v : square_vertices) {
            f32 dist =
                signed_edge_function(proj_triangle[i], proj_triangle[(i + 1) % 3], back_facing, v);
            max_signed_dist = std::max(max_signed_dist, dist);
        }
        if (max_signed_dist < 0.0f) return true;
    }
    return false;
}

inline bool
triangle_square_conservative_collision(const array<vec2, 3>& proj_triangle,
                                       bool back_facing,
                                       const array<vec2, 2>& square)
{
    const vec2 square_vertices[4] = {
        square[0], square[1], { square[0].x, square[1].y }, { square[1].x, square[0].y }
    };
    for (int i = 0; i < 4; i++)
        if (point_in_triangle(proj_triangle, back_facing, square_vertices[i])) return true;

    return false;
}

inline bool
triangle_square_6seperating_collision(const array<vec2, 3>& proj_triangle,
                                      bool back_facing,
                                      const array<vec2, 2>& square)
{
    const array<vec2, 4> means = { vec2 { (square[0].x + square[1].x) / 2, square[0].y },
                                   { (square[0].x + square[1].x) / 2, square[1].y },
                                   { square[0].x, (square[0].y + square[1].y) / 2 },
                                   { square[1].x, (square[0].y + square[1].y) / 2 } };
    for (auto& v : means)
        if (point_in_triangle(proj_triangle, back_facing, v)) return true;
    return false;
}

inline bool
aabb_collision_(const vec3& b1_min, const vec3& b1_max, const vec3& b2_min, const vec3& b2_max)
{
    if ((b2_max.x - b1_min.x) * (b2_min.x - b1_max.x) > 0.0 ||
        (b2_max.y - b1_min.y) * (b2_min.y - b1_max.y) > 0.0 ||
        (b2_max.z - b1_min.z) * (b2_min.z - b1_max.z) > 0.0)
        return false;
    return true;
}

inline bool
aabb_collision_(const array<vec3, 2>& aabb1, const array<vec3, 2>& aabb2)
{
    return aabb_collision_(aabb1[0], aabb1[0], aabb2[0], aabb2[1]);
}

inline bool
aabb_collision(const array<vec3, 2>& aabb1, const array<vec3, 2>& aabb2)
{
    return (aabb1[1].x >= aabb2[0].x && aabb1[0].x <= aabb2[1].x && aabb1[1].y >= aabb2[0].y &&
            aabb1[0].y <= aabb2[1].y && aabb1[1].z >= aabb2[0].z && aabb1[0].z <= aabb2[1].z);
}

inline bool
aabb_collision(const vec3& v1, const vec3& v2, const vec3& v3, const vec3& v4)
{
    return aabb_collision({ v1, v2 }, { v3, v4 });
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

inline array<vec3, 2>
triangle_aabb(const Triangle& t)
{
    vec3 min, max;
    ;
    min = { std::min({ t[0].x, t[1].x, t[2].x }), std::min({ t[0].y, t[1].y, t[2].y }),
            std::min({ t[0].z, t[1].z, t[2].z }) };
    max = { std::max({ t[0].x, t[1].x, t[2].x }), std::max({ t[0].y, t[1].y, t[2].y }),
            std::max({ t[0].z, t[1].z, t[2].z }) };
    return { min, max };
}

bool
is_point_in_square(const array<vec2, 2>& square, const vec2& v)
{
    return (v.x >= square[0].x && v.x <= square[1].x && v.y >= square[0].y && v.y <= square[1].y);
}

bool
is_point_in_aabb(const array<vec3, 2>& aabb, const vec3& v)
{
    return (v.x >= aabb[0].x && v.y >= aabb[0].y && v.z >= aabb[0].z && v.x <= aabb[1].x &&
            v.y <= aabb[1].y && v.z <= aabb[1].z);
}

using Intersection_Fun = bool (*)(const array<vec2, 3>&, bool, const array<vec2, 2>&);

inline std::pair<bool, f32>
triangle_aabb_collision(Triangle t, array<vec3, 2> aabb, Intersection_Option option)
{
    bool intersecting = false;
    vec3 triangle_normal = swizzle(normal(t), 2);
    for (int i = 0; i < 3; i++) {
        array<vec2, 3> proj_triangle = { swizzle(t[0], i), swizzle(t[1], i), swizzle(t[2], i) };
        array<vec2, 2> proj_aabb = { swizzle(aabb[0], i), swizzle(aabb[1], i) };
        intersecting = !has_seperating_plane(proj_triangle, triangle_normal[i] > 0, proj_aabb);
        if (intersecting) break;
    }

    // TODO: line-plane intersections beetween box edges and triangle's plane, if the triangle is not
    // inside the aabb
    f32 max_clipped_triangle_z;
    if (intersecting && option == Intersection_Option::RETURN_INTERSECTOIN_POINT) {
        std::vector<vec3> intersections(t.cbegin(), t.cend());
        intersections.reserve(9);

        // for (int axis = 0; axis < 3; axis++) {
        //     for (auto& v : t) v = swizzle(v);
        //     for (auto& v : intersections) v = swizzle(v);
        //     for (auto& v : aabb) v = swizzle(v);
        //     array<vec2, 4> square_vertices = {
        //         aabb[0], { aabb[1].x, aabb[0].y }, aabb[1], { aabb[0].x, aabb[1].y }
        //     };

        //     for (int i = 0; i < 3; i++) {
        //         array<vec2, 2> edge = { t[i], t[(i + 1) % 3] };

        //         std::vector<f32> intersections_coe;
        //         intersections_coe.reserve(2);
        //         for (size_t k = 0; k < 4; k++) {
        //             array<vec2, 2> square_edge = { square_vertices[k], square_vertices[(k + 1) % 4] };
        //             auto p = line_intersection(edge, square_edge);
        //             bool are_only_touching = (p.second.x <= 0.0f && p.second.x >= 1.0f);
        //             if (p.first && !are_only_touching) intersections_coe.push_back(p.second.x);
        //         }

        //         for (auto&& c : intersections_coe) intersections.push_back(t[0] + c * (t[1] - t[0]));
        //     }
        // }

        const array<vec3, 8> aabb_vertices = {
            vec3 { aabb[0].x, aabb[0].y, aabb[1].z }, { aabb[1].x, aabb[0].y, aabb[1].z },
            { aabb[1].x, aabb[1].y, aabb[1].z },      { aabb[0].x, aabb[1].y, aabb[1].z },
            { aabb[0].x, aabb[0].y, aabb[0].z },      { aabb[1].x, aabb[0].y, aabb[0].z },
            { aabb[1].x, aabb[1].y, aabb[0].z },      { aabb[0].x, aabb[1].y, aabb[0].z }
        };

        array<vec3, 2> aabb_edges[] = {
            { aabb[0], { aabb[1].x, aabb[0].y, aabb[0].z } },
            { aabb[0], { aabb[0].x, aabb[1].y, aabb[0].z } },
            { aabb[0], { aabb[0].x, aabb[0].y, aabb[1].z } },

            { aabb[1], { aabb[0].x, aabb[1].y, aabb[1].z } },
            { aabb[1], { aabb[1].x, aabb[0].y, aabb[1].z } },
            { aabb[1], { aabb[1].x, aabb[1].y, aabb[0].z } },

            { vec3 { aabb[0].x, aabb[0].y, aabb[1].z }, { aabb[0].x, aabb[1].y, aabb[1].z } },
            { vec3 { aabb[0].x, aabb[0].y, aabb[1].z }, { aabb[1].x, aabb[0].y, aabb[1].z } },

            { vec3 { aabb[1].x, aabb[1].y, aabb[0].z }, { aabb[1].x, aabb[0].y, aabb[0].z } },
            { vec3 { aabb[1].x, aabb[1].y, aabb[0].z }, { aabb[0].x, aabb[1].y, aabb[0].z } },

            { vec3 { aabb[0].x, aabb[1].y, aabb[0].z }, { aabb[0].x, aabb[1].y, aabb[1].z } },
            { vec3 { aabb[1].x, aabb[0].y, aabb[0].z }, { aabb[1].x, aabb[0].y, aabb[1].z } }
        };
        // std::vector<vec3> intersections(t.cbegin(), t.cend());
        // intersections.reserve(9);
        // FIXME: line-triangle algorithm between triangle's lines and the planes of aabb
        for (auto&& e : aabb_edges) {
            auto intersection = line_triangle_intersection(t, e);
            if (intersection.first) intersections.push_back(intersection.second);
        }

        array<size_t, 36> aabb_indices = { 0, 1, 2, 2, 3, 0, 1, 5, 6, 6, 2, 1, 7, 6, 5, 5, 4, 7,
                                           4, 0, 3, 3, 7, 4, 4, 5, 1, 1, 0, 4, 3, 2, 6, 6, 7, 3 };

        for (int ti = 0; ti < 3; ti++) {
            array<vec3, 2> edge = { t[ti], t[(ti + 1) % 3] };
            // for (size_t ei = 0; ei < aabb_indices.size(); ei += 3) {
            for (auto&& ei = std::begin(aabb_indices); ei != std::end(aabb_indices); ei += 3) {
                array<vec3, 3> t = { aabb_vertices[ei[0]], aabb_vertices[ei[1]], aabb_vertices[ei[2]] };
                // array<vec3, 3> t = { aabb_vertices.at(ei), aabb_vertices.at(ei + 1),
                // aabb_vertices.at(ei + 2) };
                auto intersection = line_triangle_intersection(t, edge);
                if (intersection.first) intersections.push_back(intersection.second);
            }
        }

        max_clipped_triangle_z = -1.0f;
        // max_clipped_triangle_z = aabb[0].z;
        bool all_points_outside = true, all_points_inside = true;
        for (auto& v : intersections) {
            if (is_point_in_aabb(aabb, v)) {
                all_points_outside = false;
                max_clipped_triangle_z = std::max(max_clipped_triangle_z, v.z);
            } else
                all_points_inside = false;
        }
    }

    return { intersecting, max_clipped_triangle_z };
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
