#pragma once

#include "types.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <tuple>
#include <utility>

const f32 epsilon = 0.000001;

template <typename Vec>
inline Vec
swizzle(Vec v, int n = 1)
{
    if (!n) return v;

    for (typename Vec::size_type i = 0; i < v.size() - 1; i++) std::swap(v[i], v[(i + 1) % v.size()]);
    return swizzle(v, n - 1);
}

inline bool
is_rat(f32 x)
{
    return x >= 0.0f && x <= 1.0f;
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
    // return { b != 0 && t >= 0 && t <= 1 && u >= 0 && u <= 1, intersection_point };
    return { b != 0 && is_rat(t) && is_rat(u), b ? vec2(t, u) : vec2(-1.0f, -1.0f) };
}

inline std::pair<bool, vec2>
line_intersection(const array<vec2, 2>& e1, const array<vec2, 2>& e2)
{
    return line_intersection(e1[0], e1[1], e2[0], e2[1]);
}

inline bool
is_point_in_triangle(const array<vec2, 3>& proj_triangle, bool back_facing, const vec2& p)
{
    bool result = true;
    const array<array<vec2, 2>, 3> edges = { array<vec2, 2> { proj_triangle[0], proj_triangle[1] },
                                             { proj_triangle[1], proj_triangle[2] },
                                             { proj_triangle[2], proj_triangle[0] } };

    for (auto& e : edges) result &= signed_edge_function(e[0], e[1], back_facing, p) >= 0;

    return result;
}

template <bool Print>
bool
is_point_in_aabb(const array<vec3, 2>& aabb, const vec3& v)
{
    if (Print) printf("aabbb: %s p: %s\n", aabb[0].to_string().c_str(), v.to_string().c_str());
    return (v.x >= aabb[0].x && v.y >= aabb[0].y && v.z >= aabb[0].z && v.x <= aabb[1].x &&
            v.y <= aabb[1].y && v.z <= aabb[1].z);
}

inline vector<vec3>
line_aabb_intersection(const array<vec3, 2> aabb, const array<vec3, 2>& line)
{
    vector<vec3> intersections;

    for (int axis = 0; axis < 3; axis++) {
        int maxi = (line[0][axis] > line[1][axis]) ? 0 : 1;
        int mini = (line[0][axis] < line[1][axis]) ? 0 : 1;

        f32 p;
        if (line[mini][axis] <= aabb[0][axis] && aabb[0][axis] <= line[maxi][axis])
            p = aabb[0][axis];
        else if (line[mini][axis] <= aabb[1][axis] && aabb[1][axis] <= line[maxi][axis])
            p = aabb[1][axis];
        else
            continue;

        if (line[0][axis] == line[1][axis]) {
            auto b1 = is_point_in_aabb<false>(aabb, line[0]);
            auto b2 = is_point_in_aabb<false>(aabb, line[1]);
            if (b1 && b2) {
                intersections.push_back(line[0]);
                intersections.push_back(line[1]);
            } else if (b1 != b2) {
                if (axis != 2)
                    intersections.push_back(line[0]);
                else {
                    if (p == aabb[1][axis])
                        intersections.push_back(aabb[1]);
                    else
                        intersections.push_back(line[maxi]);
                }
            }
        }

        f32 t = (p - line[0][axis]) / (line[1][axis] - line[0][axis]);
        vec3 point_on_line = line[0] + t * (line[1] - line[0]);
        if (is_point_in_aabb<false>(aabb, point_on_line)) intersections.push_back(point_on_line);
    }

    return intersections;
}

inline std::pair<bool, f32>
line_plane_intersection(const array<vec3, 3>& plane, const array<vec3, 2>& line)
{
    vec3 normal = cross(plane[1] - plane[0], plane[2] - plane[0]);
    f32 denom = dot(normal, line[1] - line[0]);
    f32 u = dot(normal, plane[0] - line[0]);
    if (!denom) {
        bool is_line_in_plane = (u == 0.0f) && (dot(normal, plane[0] - line[1]) == 0.0f);
        return { is_line_in_plane, -1.0f };
    }

    f32 t = u / denom;
    return { is_rat(t), t };
}

inline vec3
get_barycentrics_fast(const Triangle& t, const vec3& p)
{
    vec3 v0 = t[1] - t[0], v1 = t[2] - t[0], v2 = p - t[0];
    f32 d00 = dot(v0, v0);
    f32 d01 = dot(v0, v1);
    f32 d11 = dot(v1, v1);
    f32 d20 = dot(v2, v0);
    f32 d21 = dot(v2, v1);
    f32 denom = d00 * d11 - d01 * d01;
    assert(denom);
    f32 v = (d11 * d20 - d01 * d21) / denom;
    f32 w = (d00 * d21 - d01 * d20) / denom;
    f32 u = 1.0f - v - w;
    return { v, w, u };
}

inline vec3
get_barycentrics(const Triangle& t, const vec3& p)
{
    vec3 bary;
    vec3 normal = cross(t[1] - t[0], t[2] - t[0]);
    f32 areaABC = dot(normal, cross((t[1] - t[0]), (t[2] - t[0])));
    f32 areaPBC = dot(normal, cross((t[1] - p), (t[2] - p)));
    f32 areaPCA = dot(normal, cross((t[2] - p), (t[0] - p)));
    assert(areaABC);
    bary.x = areaPBC / areaABC;
    bary.y = areaPCA / areaABC;
    bary.z = 1.0f - bary.x - bary.y;

    return bary;
}

inline bool
is_point_inside_triangle(const Triangle& t, const vec3& p)
{
    vec3 bary = get_barycentrics(t, p);
    return std::all_of(std::begin(bary), std::end(bary), is_rat);
}

inline vec3
lerp(const vec3& v0, const vec3& v1, f32 t)
{
    return v0 + t * (v1 - v0);
}

inline vec3
lerp(const array<vec3, 2>& l, f32 t)
{
    return lerp(l[0], l[1], t);
}

inline std::pair<bool, vec3>
line_triangle_intersection(const Triangle& triangle, const array<vec3, 2>& line, bool* err)
{
    static bool degenerated_triangle_already_errored = false;
    static bool bary_already_errored = false;
    if (!degenerated_triangle_already_errored &&
        !(triangle[0] != triangle[1] && triangle[1] != triangle[2] && triangle[0] != triangle[2])) {
        printf("degenerated triangle detected, could cause flood-fill error\n");
        degenerated_triangle_already_errored = true;
        *err = true;
    }

    auto plane_coll = line_plane_intersection(triangle, line);
    if (!plane_coll.first) { return { false, {} }; }

    bool intersecting = false;
    vec3 p;
    if (!(plane_coll.second < 0.0f)) {
        assert(is_rat(plane_coll.second));
        p = lerp(line, plane_coll.second);
        intersecting = is_point_inside_triangle(triangle, p);
    } else { // the line lies on t's plane, taking both intersections with tri
        vec3 bary_l0 = get_barycentrics(triangle, line[0]);
        vec3 bary_l1 = get_barycentrics(triangle, line[1]);
        bool is_l0_in_triangle = std::all_of(bary_l0.begin(), bary_l0.end(), is_rat);
        bool is_l1_in_triangle = std::all_of(bary_l1.begin(), bary_l1.end(), is_rat);
        if (is_l0_in_triangle && is_l1_in_triangle) {
            intersecting = true;
            p = (line[0].z < line[1].z ? line[1] : line[0]);
        } else {
            // p = l0 + t * (l1-l0) => t = l0 / (l0 - l1) in the triangle's edges
            vec3 t = bary_l0 / (bary_l0 - bary_l1);
            array<vec3, 2> intersections = {};
            for (auto& x : t) {
                if (std::isnan(x)) continue;

                vec3 bary = lerp(bary_l0, bary_l1, x);
                if (std::any_of(bary.begin(), bary.end(), [](f32 x) { return !is_rat(x); })) continue;

                intersections[intersecting ? 1 : 0] =
                    (bary.x * triangle[0] + bary.y * triangle[1] + bary.z * triangle[2]);
                intersecting = true;
            }
            assert(intersections.size() < 3);
            p = (intersections[0].z < intersections[1].z) ? intersections[1] : intersections[0];

            auto is_line_crossing = is_l0_in_triangle != is_l1_in_triangle;
            if (!bary_already_errored && !intersecting && is_line_crossing) {
                printf(
                    "Error: the line found intersecting with the triangle and lies on its plane but no intersections found\n");
                bary_already_errored = true;
                *err = true;
            }
        }
    }

    return { intersecting, p };
}

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
        if (is_point_in_triangle(proj_triangle, back_facing, square_vertices[i])) return true;

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
        if (is_point_in_triangle(proj_triangle, back_facing, v)) return true;
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
    min = { std::min({ t[0].x, t[1].x, t[2].x }), std::min({ t[0].y, t[1].y, t[2].y }),
            std::min({ t[0].z, t[1].z, t[2].z }) };
    max = { std::max({ t[0].x, t[1].x, t[2].x }), std::max({ t[0].y, t[1].y, t[2].y }),
            std::max({ t[0].z, t[1].z, t[2].z }) };
    return { min, max };
}

inline bool
is_point_in_square(const array<vec2, 2>& square, const vec2& v)
{
    return (v.x >= square[0].x && v.x <= square[1].x && v.y >= square[0].y && v.y <= square[1].y);
}

inline bool
triangle_aabb_collision(const Triangle& t, const array<vec3, 2>& aabb)
{
    bool intersecting = false;
    vec3 triangle_normal = swizzle(normal(t), 2);
    for (int i = 0; i < 3; i++) {
        array<vec2, 3> proj_triangle = { swizzle(t[0], i), swizzle(t[1], i), swizzle(t[2], i) };
        array<vec2, 2> proj_aabb = { swizzle(aabb[0], i), swizzle(aabb[1], i) };
        if (has_seperating_plane(proj_triangle, triangle_normal[i] > 0, proj_aabb)) return false;
    }
    return true;
}

inline array<vec3, 2>
get_best_diag(const Triangle& t, const array<vec3, 2>& aabb)
{
    vec3 aabb_center = (aabb[1] - aabb[0]) / 2.0f;
    Triangle triangle_aabb_space;
    std::transform(t.cbegin(), t.cend(), triangle_aabb_space.begin(),
                   [&](const vec3& v) { return v - aabb_center; });
    vec3 triangle_normal = cross(triangle_aabb_space[1] - triangle_aabb_space[0],
                                 triangle_aabb_space[2] - triangle_aabb_space[0]);
    array<vec3, 2> best_diag;
    for (int axis = 0; axis < 3; axis++) {
        int i = triangle_normal[axis] > 0.0f ? 1 : 0;
        best_diag[1][axis] = aabb[i][axis];
        best_diag[0][axis] = aabb[i ? 0 : 1][axis];
    }

    return best_diag;
}

inline vector<vec3>
find_triangle_aabb_collision(const Triangle& t, const array<vec3, 2>& aabb, bool verbose = false)
{
    // TODO: check the indices
    const array<vec3, 8> aabb_vertices = {
        vec3 { aabb[0].x, aabb[0].y, aabb[1].z }, { aabb[1].x, aabb[0].y, aabb[1].z },
        { aabb[1].x, aabb[1].y, aabb[1].z },      { aabb[0].x, aabb[1].y, aabb[1].z },
        { aabb[0].x, aabb[0].y, aabb[0].z },      { aabb[1].x, aabb[0].y, aabb[0].z },
        { aabb[1].x, aabb[1].y, aabb[0].z },      { aabb[0].x, aabb[1].y, aabb[0].z }
    };

    array<size_t, 36> aabb_indices = { 0, 1, 2, 2, 3, 0, 1, 5, 6, 6, 2, 1, 7, 6, 5, 5, 4, 7,
                                       4, 0, 3, 3, 7, 4, 4, 5, 1, 1, 0, 4, 3, 2, 6, 6, 7, 3 };

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

    const array<vec3, 2> aabb_diags[4] = {
        { aabb[0], aabb[1] },
        { vec3 { aabb[0].x, aabb[1].y, aabb[0].z }, { aabb[0].x, aabb[1].y, aabb[1].z } },
        { vec3 { aabb[1].x, aabb[0].y, aabb[0].z }, { aabb[1].x, aabb[0].y, aabb[1].z } },
        { vec3 { aabb[1].x, aabb[1].y, aabb[0].z }, { aabb[1].x, aabb[1].y, aabb[1].z } }
    };

    vector<vec3> intersections;
    intersections.reserve(6);

    bool err = false;
    for (auto& e : aabb_edges) {
        auto intersection = line_triangle_intersection(t, e, &err);
        if (intersection.first) intersections.push_back(intersection.second);
        if (err) {
            err = false;
            printf("aabb: %s\n", aabb[0].to_string().c_str());
        }
    }

    array<vec3, 2> best_diag = get_best_diag(t, aabb);
    for (auto& d : aabb_diags) {
        auto colls = line_aabb_intersection(aabb, d);
        intersections.insert(intersections.cend(), colls.cbegin(), colls.cend());
    }

    if (intersections.empty()) {
        for (int ti = 0; ti < 3; ti++) {
            array<vec3, 2> edge = { t[ti], t[(ti + 1) % 3] };
            auto inter = line_aabb_intersection(aabb, edge);
            intersections.insert(intersections.cend(), inter.cbegin(), inter.cend());
        }
    }

    return intersections;
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
