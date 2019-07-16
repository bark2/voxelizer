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
// if flood-fill, then store the non-normalized directions of the edges and the orig

struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    bool has_uvs;
};

// TODO: split to two types, while the other contains *only* the type
struct Voxel {
    bool valid;
    enum Type { NONE, CLOSING, OPENING, BOTH } max_type;
    f64 max_intersection_off;
};
