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
swizzle(Vec v, int n)
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

inline std::pair<bool, f32>
line_intersection(
    const vec2& v1, const vec2& v2, const vec2& v3, const vec2& v4, bool* return_info = nullptr)
{
    auto cross = [](const vec2& v1, const vec2 v2) { return v1.x * v2.y - v1.y * v2.x; };
    vec2 a = v3 - v1;
    f32 b = cross(v2 - v1, v4 - v3);
    f32 t = cross(a, v4 - v3) / b;
    f32 u = cross(a, v2 - v1) / b;
    vec2 intersection_point = v1 + t * (v2 - v1);
    if (return_info) *return_info = (t == 0.0f || t == 1.0f || u == 0.0f || u == 1.0f);
    // return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, intersection_point };
    return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, t };
}

inline std::pair<bool, f32>
line_intersection(const array<vec2, 2>& e1, const array<vec2, 2>& e2, bool* return_info = nullptr)
{
    return line_intersection(e1[0], e1[1], e2[0], e2[1], return_info);
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

// only for aabbs that colliding with t's aabb?
inline bool
triangle_square_fconservative_collision(const array<vec2, 3>& proj_triangle,
                                        bool back_facing,
                                        const array<vec2, 2>& square)
{
    const array<vec2, 4> square_vertices = {
        square[0], square[1], { square[0].x, square[1].y }, { square[1].x, square[0].y }
    };
    // checks for each edge, if one of the square vertices is above it
    // also holds if the triangle is inside the square
    for (i32 i = 0; i < 3; i++) {
        f32 max_signed_dist = -1.0f;
        for (auto& v : square_vertices) {
            f32 dist =
                signed_edge_function(proj_triangle[i], proj_triangle[(i + 1) % 3], back_facing, v);
            max_signed_dist = std::max(max_signed_dist, dist);
        }
        if (max_signed_dist < 0.0f) return false;
    }
    return true;
}

inline bool
triangle_square_conservative_collision(const array<vec2, 3>& proj_triangle,
                                       bool back_facing,
                                       const array<vec2, 2>& square)
{
    const vec2 square_vertices[4] = {
        square[0], square[1], { square[0].x, square[1].y }, { square[1].x, square[0].y }
    };
    bool proj_voxel_collision = false;
    for (int i = 0; i < 4; i++)
        proj_voxel_collision |= point_in_triangle(proj_triangle, back_facing, square_vertices[i]);

    return proj_voxel_collision;
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
    bool proj_voxel_collision = false;
    for (auto& v : means) proj_voxel_collision |= point_in_triangle(proj_triangle, back_facing, v);
    return proj_voxel_collision;
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
    return (v.x > square[0].x && v.x < square[1].x && v.y > square[0].y && v.y < square[1].y);
}

using Intersection_Fun = bool (*)(const array<vec2, 3>&, bool, const array<vec2, 2>&);

inline std::pair<bool, f32>
triangle_aabb_collision(Triangle t,
                        array<vec3, 2> aabb,
                        Intersection_Fun intersection_fun,
                        Intersection_Option option)
{
    bool intersecting = false;
    vec3 triangle_normal = swizzle(normal(t), 2);
    for (int i = 0; i < 3; i++) {
        array<vec2, 3> proj_triangle = { swizzle(t[0], i), swizzle(t[1], i), swizzle(t[2], i) };
        array<vec2, 2> proj_aabb = { swizzle(aabb[0], i), swizzle(aabb[1], i) };
        intersecting |= intersection_fun(proj_triangle, triangle_normal[i] > 0, proj_aabb);
        if (intersecting) break;
    }

    // find out which triangles pass the collision test for the requested function,
    // then better optimize that
    f32 max_clipped_triangle_z;
    if (intersecting && option == Intersection_Option::RETURN_INTERSECTOIN_POINT) {
        std::vector<vec3> clipped(t.cbegin(), t.cend());
        clipped.reserve(9);

        for (int axis = 0; axis < 3; axis++) {
            for (auto& v : clipped) v = swizzle(v, 1);
            for (auto& v : aabb) v = swizzle(v, 1);
            array<vec2, 4> square_vertices = {
                aabb[0], { aabb[1].x, aabb[0].y }, aabb[1], { aabb[0].x, aabb[1].y }
            };
            int zi = 2 - axis;

            for (std::size_t i = 0; i < clipped.size(); i++) {
                array<vec2, 2> edge = { clipped[i], clipped[(i + 1) % clipped.size()] };

                std::vector<f32> intersections_coe;
                intersections_coe.reserve(2);
                for (int k = 0; k < 4; k++) {
                    array<vec2, 2> square_edge = { square_vertices[k], square_vertices[(k + 1) % 4] };
                    bool are_only_touching;
                    auto p = line_intersection(edge, square_edge, &are_only_touching);
                    if (p.first && !are_only_touching) intersections_coe.push_back(p.second);
                }

                std::sort(intersections_coe.begin(), intersections_coe.end());
                std::vector<vec3> intersections;
                intersections.reserve(intersections_coe.size());
                for (auto&& t : intersections_coe)
                    intersections.push_back(clipped[i] +
                                            t * (clipped[(i + 1) % clipped.size()] - clipped[i]));
                clipped.insert(clipped.begin() + i + 1, intersections.cbegin(), intersections.cend());
            }
        }

        max_clipped_triangle_z = -1.0f;
        // zxy
        for (auto&& v : clipped)
            if (v.x >= aabb[0].z && v.y >= aabb[0].x && v.z >= aabb[0].y && v.x <= aabb[1].z &&
                v.y <= aabb[1].x && v.z <= aabb[1].y)
                max_clipped_triangle_z = std::max(max_clipped_triangle_z, v.x);
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
