#pragma once

#include "types.h"
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <limits>
#include <tuple>
#include <utility>

const f64 epsilon = 1.0E-9f;

template <typename Vec>
inline Vec
swizzle(Vec v, int n = 1)
{
    if (!n) return v;

    for (typename Vec::size_type i = 0; i < v.size() - 1; i++) std::swap(v[i], v[(i + 1) % v.size()]);
    return swizzle(v, n - 1);
}

inline bool
is_rat(f64 x)
{
    return x >= 0.0f && x <= 1.0f;
}

inline f64
signed_edge_function(const vec2& v0, const vec2& v1, const bool back_facing, const vec2& test_point)
{
    f64 d = back_facing ? 1.0f : -1.0f;
    vec2 edge = v1 - v0;
    vec2 edge_normal = d * vec2(-edge.y, edge.x);
    return dot(edge_normal, test_point - v0);
}

inline f64
signed_edge_function(const array<vec2, 2>& e1, const bool back_facing, const vec2& test_point)
{
    return signed_edge_function(e1[0], e1[1], back_facing, test_point);
}

template <typename T> struct Collision {
    // bool is_coll;
    // T coll;
    union {
        struct {
            bool _valid;
            T second;
        };
        bool first;
    };
};

inline std::pair<bool, vec2>
line_intersection(const vec2& v1, const vec2& v2, const vec2& v3, const vec2& v4)
{
    auto cross = [](const vec2& v1, const vec2 v2) { return v1.x * v2.y - v1.y * v2.x; };
    vec2 a = v3 - v1;
    f64 b = cross(v2 - v1, v4 - v3);
    f64 t = cross(a, v4 - v3) / b;
    f64 u = cross(a, v2 - v1) / b;
    // vec2 intersection_point = v1 + t * (v2 - v1);
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

bool
is_point_in_aabb(const array<vec3, 2>& aabb, const vec3& v)
{
    // if (Print) printf("aabbb: %s p: %s\n", aabb[0].to_string().c_str(), v.to_string().c_str());
    return (v.x >= aabb[0].x && v.y >= aabb[0].y && v.z >= aabb[0].z && v.x <= aabb[1].x &&
            v.y <= aabb[1].y && v.z <= aabb[1].z);
}

inline vec3
lerp(const vec3& v0, const vec3& v1, f64 t)
{
    // assert(is_rat(t));
    return v0 + t * (v1 - v0);
}

inline vec3
lerp(const array<vec3, 2>& l, f64 t)
{
    return lerp(l[0], l[1], t);
}

inline std::pair<bool, f64>
// inline Collision<f64>
line_plane_intersection(const vec3& normal, const vec3& point, const array<vec3, 2>& line)
{
    f64 denom = dot(normal, line[1] - line[0]);
    f64 u = dot(normal, point - line[0]);
    if (std::abs(denom) < epsilon) return { std::abs(u) < epsilon, -1.0f };

    f64 t = u / denom;
    return { is_rat(t), t };
}

inline std::pair<bool, f64>
line_plane_intersection(const array<vec3, 3>& plane, const array<vec3, 2>& line)
{
    vec3 normal = cross(plane[1] - plane[0], plane[2] - plane[0]);
    return line_plane_intersection(normal, plane[0], line);
}

// verifed fast and regular version work are identical
inline std::pair<bool, vec2>
line_aabb_intersection_fast(const array<vec3, 2> aabb, const array<vec3, 2>& line)
{
    using std::max;
    using std::min;

    vec3 dir = line[1] - line[0];
    // r.dir is unit direction vector of ray
    vec3 inv_dir;
    inv_dir.x = 1.0f / dir.x;
    inv_dir.y = 1.0f / dir.y;
    inv_dir.z = 1.0f / dir.z;

    f64 t1 = (aabb[0].x - line[0].x) * inv_dir.x;
    f64 t2 = (aabb[1].x - line[0].x) * inv_dir.x;
    f64 t3 = (aabb[0].y - line[0].y) * inv_dir.y;
    f64 t4 = (aabb[1].y - line[0].y) * inv_dir.y;
    f64 t5 = (aabb[0].z - line[0].z) * inv_dir.z;
    f64 t6 = (aabb[1].z - line[0].z) * inv_dir.z;

    f64 tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
    f64 tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

    vec2 t = { tmin, tmax };
    if (tmax < 0 || tmin > tmax) return { false, t };

    return { true, t };
}

inline vector<vec3>
line_aabb_intersection(const array<vec3, 2> aabb, const array<vec3, 2>& line)
{
    vector<vec3> intersections;
    intersections.reserve(2);

    vec3 normal = { 1.0f, 0.0f, 0.0f };
    for (int axis = 0; axis < 3; axis++) {
        auto coll_far = line_plane_intersection(normal, aabb[1], line);
        auto coll_near = line_plane_intersection(normal, aabb[0], line);
        for (auto& coll : { coll_near, coll_far }) {
            if (coll.first) {
                // FIXME robustness
                auto p = lerp(line, coll.second);
                if (is_point_in_aabb(aabb, p)) intersections.push_back(p);
            }
        }
        normal = swizzle(normal);
    }

    return intersections;
}

inline vec3
get_barycentrics_fast(const Triangle& t, const vec3& p)
{
    vec3 v0 = t[1] - t[0], v1 = t[2] - t[0], v2 = p - t[0];
    f64 d00 = dot(v0, v0);
    f64 d01 = dot(v0, v1);
    f64 d11 = dot(v1, v1);
    f64 d20 = dot(v2, v0);
    f64 d21 = dot(v2, v1);
    f64 denom = d00 * d11 - d01 * d01;
    assert(denom);
    f64 v = (d11 * d20 - d01 * d21) / denom;
    f64 w = (d00 * d21 - d01 * d20) / denom;
    f64 u = 1.0f - v - w;
    return { v, w, u };
}

inline vec3
get_barycentrics(const Triangle& t, const vec3& p)
{
    vec3 bary;
    vec3 normal = cross(t[1] - t[0], t[2] - t[0]);
    f64 areaABC = dot(normal, cross((t[1] - t[0]), (t[2] - t[0])));
    f64 areaPBC = dot(normal, cross((t[1] - p), (t[2] - p)));
    f64 areaPCA = dot(normal, cross((t[2] - p), (t[0] - p)));
    assert(areaABC);
    bary.x = areaPBC / areaABC;
    bary.y = areaPCA / areaABC;
    bary.z = 1.0f - bary.x - bary.y;

    return bary;
}

inline std::pair<bool, f64>
line_triangle_intersection_mt(const array<vec3, 2>& line, const Triangle& tri)
{
    // FIXME unit?
    vec3 dir = unit(line[1] - line[0]);
    // vec3 dir = (line[1] - line[0]);
    vec3 e1 = tri[1] - tri[0];
    vec3 e2 = tri[2] - tri[0];
    vec3 pvec = cross(dir, e2);

    f64 u = dot(line[1] - line[0], tri[0] - line[0]);

    f64 denom = dot(e1, pvec);
    if (std::abs(denom) < epsilon) return { std::abs(u) < epsilon, -1.0f };

    f64 inv_denom = 1 / denom;
    vec3 tvec = line[0] - tri[0];
    vec3 bary;
    bary[1] = dot(tvec, pvec) * inv_denom;
    if (!is_rat(bary[1])) return { false, {} };

    vec3 qvec = cross(tvec, e1);
    bary[2] = dot(dir, qvec) * inv_denom;
    if (bary[2] < 0.0f || bary[1] + bary[2] > 1.0f) return { false, {} };

    auto t = dot(e2, qvec) * inv_denom;
    return { true, t };
}

inline std::pair<bool, vec3>
line_triangle_intersection(const array<vec3, 2>& line, const Triangle& triangle)
{
    auto plane_coll = line_plane_intersection(triangle, line);
    if (!plane_coll.first) { return { false, {} }; }

    bool intersecting = false;
    vec3 p;
    if (plane_coll.second >= 0.0f) {
        p = lerp(line, plane_coll.second);
        auto bary = get_barycentrics(triangle, p);
        intersecting = std::all_of(bary.begin(), bary.end(), is_rat);
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
                if (std::any_of(bary.begin(), bary.end(), [](f64 x) { return !is_rat(x); })) continue;

                intersections[intersecting ? 1 : 0] =
                    (bary.x * triangle[0] + bary.y * triangle[1] + bary.z * triangle[2]);
                intersecting = true;
            }
            assert(intersections.size() < 3);
            p = (intersections[0].z < intersections[1].z) ? intersections[1] : intersections[0];

            auto is_line_crossing = is_l0_in_triangle != is_l1_in_triangle;

            static bool bary_already_errored = false;
            if (!bary_already_errored && !intersecting && is_line_crossing) {
                printf(
                    "Error: the line found intersecting with the triangle and lies on its plane but no intersections found\n");
                bary_already_errored = true;
            }
        }
    }
    return { intersecting, p };
}

// inline std::pair<bool, vec3>
inline Collision<vec3>
line_triangle_intersection_fast(const array<vec3, 2>& line, const Triangle& triangle)
{
    auto coll = line_triangle_intersection_mt(line, triangle);
    if (coll.first && coll.second != 1.0f) {
        if (coll.second < 0.0f || coll.second > (line[1] - line[0]).length()) return { .first = false };
        return { true, lerp(line, coll.second) };
    } else if (coll.second != -1.0f)
        return { false, {} };

    bool intersecting;
    vec3 p;
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
            if (std::any_of(bary.begin(), bary.end(), [](f64 x) { return !is_rat(x); })) continue;

            intersections[intersecting ? 1 : 0] =
                (bary.x * triangle[0] + bary.y * triangle[1] + bary.z * triangle[2]);
            intersecting = true;
        }
        assert(intersections.size() < 3);
        p = (intersections[0].z < intersections[1].z) ? intersections[1] : intersections[0];

        auto is_line_crossing = is_l0_in_triangle != is_l1_in_triangle;
        assert(is_line_crossing && intersections.size() == 2);
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
        array<vec2, 2> edge = { proj_triangle[i], proj_triangle[(i + 1) % 3] };
        f64 max_signed_dist = -std::numeric_limits<f64>::max();
        for (auto& v : square_vertices) {
            f64 dist = signed_edge_function(edge, back_facing, v);
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
aabb_collision(const vec3& b1_min, const vec3& b1_max, const vec3& b2_min, const vec3& b2_max)
{
    if ((b2_max.x - b1_min.x) * (b2_min.x - b1_max.x) > 0.0 ||
        (b2_max.y - b1_min.y) * (b2_min.y - b1_max.y) > 0.0 ||
        (b2_max.z - b1_min.z) * (b2_min.z - b1_max.z) > 0.0)
        return false;
    return true;
}

inline bool
aabb_collision(const array<vec3, 2>& aabb1, const array<vec3, 2>& aabb2)
{
    return aabb_collision(aabb1[0], aabb1[0], aabb2[0], aabb2[1]);
}

inline i32
dominant_axis(const vec3& tnormal, array<vec3, 3> axises)
{
    array<f64, 3> normal_projections = { std::abs(dot(axises[0], tnormal)),
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

    // TODO: printf each's Voxel::NONE, the false intersection points
    vector<vec3> collisions;
    collisions.reserve(6);
    for (auto& e : aabb_edges) {
        auto coll = line_triangle_intersection_fast(e, t);
        if (coll.first) collisions.push_back(coll.second);

        if (verbose) {
            auto coll_ = line_triangle_intersection(e, t);
            if (coll.first && coll_.first) {
                auto delta = coll.second - coll_.second;
                if (std::any_of(delta.begin(), delta.end(), [](f64 x) { return x > epsilon; }))
                    printf(
                        "Error: inconsistency between line_triangle_intersection methods, delta: %s\n",
                        delta.to_string().c_str());
            }
        }
    }

    for (int ti = 0; ti < 3; ti++) {
        array<vec3, 2> edge = { t[ti], t[(ti + 1) % 3] };
        auto colls = line_aabb_intersection_fast(aabb, edge);
        if (colls.first) {
            if (is_rat(colls.second.x)) collisions.push_back(lerp(edge, colls.second.x));
            if (is_rat(colls.second.y)) collisions.push_back(lerp(edge, colls.second.y));
        }
    }

    return collisions;
}

inline bool
triangle_aabb_collision(const Triangle& t, const array<vec3, 2>& aabb, bool verbose = false)
{
    bool result = true;
    vec3 triangle_normal = swizzle(unit(cross(t[1] - t[0], t[2] - t[0])), 2);
    for (int i = 0; result && i < 3; i++) {
        array<vec2, 3> proj_triangle = { swizzle(t[0], i), swizzle(t[1], i), swizzle(t[2], i) };
        array<vec2, 2> proj_aabb = { swizzle(aabb[0], i), swizzle(aabb[1], i) };
        if (has_seperating_plane(proj_triangle, triangle_normal[i] > 0, proj_aabb)) result = false;
    }

    if (verbose) {
        static bool inconsistent_error = false;
        auto collisions = find_triangle_aabb_collision(t, aabb);
        if (!inconsistent_error && result != !collisions.empty()) {
            printf("Error: intersection and find function are inconsistent\n");
            inconsistent_error = true;
        }
    }

    return result;
}

inline f64
progressive_floor(f64 f)
{
    return std::max(static_cast<f64>(0.0f), f == std::floor(f) ? f - 1 : std::floor(f));
}

inline f64
progressive_ceil(f64 f, f64 max)
{
    return std::min(max, f == std::ceil(f) ? f + 1 : std::ceil(f));
}
