#pragma once

#include <array>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <string>
#include <vector>

using u8 = unsigned char;
using u32 = uint32_t;
using i32 = int32_t;
using f32 = float;
using f64 = double;
using glm::vec3;
using glm::vec2;
using std::array;

struct Vertex {
    vec3 pos;
    vec3 normal;
    vec2 uv;
};

struct Triangle : public array<vec3, 3> {
    vec3 normal() const;
};




struct Mesh {
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    bool has_uvs;
};

