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
using f32 = float;
using f64 = double;

using std::array;

struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

using Triangle = array<vec3,3>;
vec3 normal(const Triangle& t);

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    bool has_uvs;
};

struct Voxel {
    bool valid;
    enum Type { NONE, CLOSING, OPENING, BOTH } max_type;
    f32 max_intersection_off;
};

