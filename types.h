#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <cassert>

#include "vec2.h"
#include "vec3.h"

using u8 = unsigned char;
using u32 = uint32_t;
using i32 = int32_t;
using f32 = float;
using f64 = double;

using std::array;

struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

struct Triangle : array<vec3, 3> {
    vec3 normal() const;
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    bool has_uvs;
};

struct Voxel {
  bool valid;
  enum Type { CLOSING, OPENING, BOTH } type;
};

template <typename Vec>
inline Vec
to_xy(const Vec& v)
{
    return { v[0], v[1], v[2] };
};
template <typename Vec>
inline Vec
to_yz(const Vec& v)
{
    return { v[1], v[2], v[0] };
};
template <typename Vec>
inline Vec
to_zx(const Vec& v)
{
    return { v[2], v[0], v[1] };
};
template <typename Vec>
inline Vec
inversed_xy(const Vec& v)
{
    return { v[0], v[1], v[2] };
};
template <typename Vec>
inline Vec
inversed_yz(const Vec& v)
{
    return { v[2], v[0], v[1] };
};
template <typename Vec>
inline Vec
inversed_zx(const Vec& v)
{
    return { v[1], v[2], v[0] };
};
template <typename Vec> using Swizzler = Vec (*)(const Vec&);

template <typename Vec>
Swizzler<Vec>
get_swizzler(u32 i)
{
    Swizzler<Vec> result;
    switch (i) {
    case 0: {
        result = to_yz;
    } break;
    case 1: {
        result = to_zx;
    }; break;
    case 2: {
        result = to_xy;
    } break;
    default:
        assert(false && "swizzling with wrong index");
    }
    return result;
}
template <typename Vec>
Swizzler<Vec>
get_inv_swizzler(u32 i)
{
    Swizzler<Vec> result;
    switch (i) {
    case 0: {
        result = inversed_yz;
    } break;
    case 1: {
        result = inversed_zx;
    }; break;
    case 2: {
        result = inversed_xy;
    } break;
    default:
        assert(false && "swizzling with wrong index");
    }
    return result;
}
