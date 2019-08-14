#pragma once

#include "csignal"
#include <array>
#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "vec2.h"
#include "vec3.h"

using u8 = unsigned char;
using u32 = uint32_t;
using i32 = int32_t;

using std::array;
using std::vector;

struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

using Triangle = array<vec3, 3>;

struct DTriangle {
    vec3 v;
    vec3 e[2];

    vec3 operator[](int i)
    {
        vec3 result;
        switch (i) {
        case 0: result = v; break;
        case 1: result = e[0] + v; break;
        case 2: result = e[1] + v; break;
        default: assert(0);
        }
        return result;
    };

    vec3
    normal()
    {
        return cross(e[0], e[1]);
    };
};

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    bool has_uvs;
};

struct Voxel {
    bool valid;
    enum Type { NONE, CLOSING, OPENING, BOTH } max_type;
    f64 max_coll_off;
    // FIXME: add left-of / right-of
};

struct Array {
    u8 elem_bits = 1;
    u8* buff = nullptr;

    inline ~Array()
    {
        assert(buff);
        free(buff);
    }

    inline void*
    at(size_t n)
    {
        return static_cast<void*>(&buff[n * elem_bits / 8]);
    }

    inline void const*
    at(size_t n) const
    {
        return static_cast<void*>(&buff[n * elem_bits / 8]);
    }
};
