#include "common.h"
#include "types.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

static const u32 max_line = 128;

enum Command {
    POSITION = 0,
    TEXTURE,
    NORMAL,
    FACE,
    GROUP,
    OBJECT,
    SMOOTH,
    COMMENT,
    COMMANDS_COUNT
};

static const std::array<const std::string, COMMANDS_COUNT> commands_map {
    "v ", "vt ", "vn ", "f ", "g ", "o ", "s ", "#"
};

std::vector<Mesh>
load_obj_file(const std::string& filename)
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cout << "Error: load_obj_file\tfile: " << __FILE__ << "\tline:" << __LINE__
                  << '\n';
        return {};
    }

    std::vector<Mesh> meshes;
    std::vector<Vertex> vertices;
    std::vector<u32> indices;
    u32 vertex_properties_n = 0;

    std::vector<vec3> positions;
    std::vector<vec3> normals;
    std::vector<vec2> uvs;
    vec3 aabb_min { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                    std::numeric_limits<float>::max() },
        aabb_max { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                   -std::numeric_limits<float>::max() };

    std::string line;
    size_t i_line = 0;
    line.reserve(max_line);
    for (u32 line_n = 0; std::getline(infile, line); i_line = 0, line_n++) {
        u32 c;
        for (c = 0; c < COMMANDS_COUNT; c++) {
            const auto& command = commands_map[c];
            if (!command.compare(0, command.length(), line, 0, command.length())) break;
        }
        if (c == (u32)COMMANDS_COUNT) {
            if (!std::all_of(line.cbegin(), line.cend(), isspace)) {
                std::cout << line_n << std::endl;
                assert(0);
            }
            continue;
        }
        i_line += commands_map[c].length();

        switch (c) {
        case POSITION: {
            vec3 v;
            for (u32 i = 0; i < 3; i++)
                v[i] = strtof(strtok_update(line, " ", i_line).data(), nullptr);
            positions.push_back(v);
            for (size_t i = 0; i < 3; i++) {
                aabb_max[i] = std::max(aabb_max[i], v[i]);
                aabb_min[i] = std::min(aabb_min[i], v[i]);
            }
        } break;
        case TEXTURE: {
            vec2 v;
            for (u32 i = 0; i < 2; i++)
                v[i] = strtof(strtok_update(line, " ", i_line).data(), nullptr);
            uvs.push_back(v);
        } break;
        case NORMAL: {
            vec3 v;
            for (u32 i = 0; i < 3; i++)
                v[i] = strtof(strtok_update(line, " ", i_line).data(), nullptr);
            normals.push_back(v);
        } break;
        case FACE: {
            if (!vertex_properties_n) {
                assert(line[i_line] != ' ');
                vertex_properties_n = 1;
                size_t end_vertex_properties = line.find_first_of(' ', i_line);
                {
                    size_t i;
                    for (i = i_line; i < end_vertex_properties; i++) {
                        if (!line.compare(i, 2, "//", 2)) {
                            vertex_properties_n = 2;
                            break;
                        }
                    }
                    if (i == end_vertex_properties) vertex_properties_n = 3;
                }
            }

            u32 is_quad = false;
            Vertex face[6] = {};
            for (u32 i = 0; i < 4 && i_line < line.length(); i++) {
                if (i == 3) {
                    is_quad = true;
                    i = 5;
                }

                u32 indexes[3];
                const size_t first_delimeter = line.find_first_of('/', i_line);
                const size_t second_delimeter
                    = line.find_first_of('/', first_delimeter + 1);
                const size_t ending_space = line.find_first_of(' ', second_delimeter + 1);

                indexes[0] = strtoul(
                                 line.substr(static_cast<size_t>(i_line), first_delimeter)
                                     .c_str(),
                                 nullptr, 10)
                    - 1;
                indexes[1] = strtoul(
                                 line.substr(
                                         static_cast<size_t>(first_delimeter + 1),
                                         second_delimeter)
                                     .c_str(),
                                 nullptr, 10)
                    - 1;
                indexes[2]
                    = strtoul(
                          line.substr(
                                  static_cast<size_t>(second_delimeter + 1), ending_space)
                              .c_str(),
                          nullptr, 10)
                    - 1;

                face[i]
                    = { positions[indexes[0]], normals[indexes[2]],
                        (vertex_properties_n == 3) ? uvs[indexes[1]] : vec2(0.0f, 0.0f) };

                for (i_line = ending_space;
                     i_line < line.length() && isspace(line.at(i_line)); i_line++)
                    ;
            }
            if (is_quad) {
                face[3] = face[0];
                face[4] = face[2];
            }

            for (u32 i = 0, n = (is_quad) ? 6 : 3; i < n; i++) {
                indices.push_back(vertices.size());
                vertices.push_back(face[i]);
            }
            // TODO: replace existing indices using a map
        } break;
        case OBJECT: {
            if (vertices.empty()) break;

            meshes.push_back({ std::move(vertices), std::move(indices),
                               vertex_properties_n == 3 ? true : false });
            vertex_properties_n = 0;
        } break;
        default:;
        } // switch(c)
    }
    infile.close();

    if (!vertices.empty())
        meshes.push_back({ std::move(vertices), std::move(indices),
                           vertex_properties_n == 3 ? true : false });

    vec3 aabb_range = aabb_max - aabb_min;
    f32 aabb_max_size = std::max({ aabb_range.x, aabb_range.y, aabb_range.z });
    for (auto& m : meshes) {
        for (auto& v : m.vertices) {
            v.pos.x = (v.pos.x - aabb_min.x) / (aabb_max.x - aabb_min.x);
            v.pos.y = (v.pos.y - aabb_min.y) / (aabb_max.y - aabb_min.y);
            v.pos.z = (v.pos.z - aabb_min.z) / (aabb_max.z - aabb_min.z);
            assert(v.pos.x <= 1 && v.pos.y <= 1 && v.pos.z <= 1);
        }
    }

    return std::move(meshes);
}

// 2. Chunk Structure
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 1x4      | char       | chunk id
// 4        | int        | num bytes of chunk content (N)
// 4        | int        | num bytes of children chunks (M)
// N        |            | chunk content
// M        |            | children chunks
// -------------------------------------------------------------------------------
// 5. Chunk id 'SIZE' : model size
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4        | int        | size x
// 4        | int        | size y
// 4        | int        | size z : gravity direction
// -------------------------------------------------------------------------------
// 6. Chunk id 'XYZI' : model voxels
// -------------------------------------------------------------------------------
// # Bytes  | Type       | Value
// -------------------------------------------------------------------------------
// 4        | int        | numVoxels (N)
// 4 x N    | int        | (x, y, z, colorIndex) : 1 byte for each component
// -------------------------------------------------------------------------------

inline u32
to_little(u32 big)
{
    u32 c1 = (big & 0xFF) << 24;
    u32 c2 = (big & (0xFF << 8)) << 8;
    u32 c3 = (big & (0xFF << 16)) >> 8;
    u32 c4 = (big & (0xFF << 24)) >> 24;
    return c1 | c2 | c3 | c4;
}

// little indian
int
export_vox_file(
    const std::string& filename,
    const std::vector<bool>& grid,
    u32 resolution,
    u32 voxels_n)
{
    FILE* out = fopen(filename.c_str(), "wb");
    if (!out) return 1;

    u32 meta_size = 4 * 3 * sizeof(i32);
    u32 size_size = 3 * sizeof(i32);
    u32 xyzi_size = (voxels_n + 1) * sizeof(i32);
    u32 main_children_size = 2 * meta_size + size_size + xyzi_size;
    u32 total = main_children_size + meta_size;
    u32 header[] = { to_little(0x564f5820),
                     150,
                     to_little(0x4d41494e),
                     0,
                     main_children_size,
                     to_little(0x53495a45),
                     size_size,
                     0,
                     resolution,
                     resolution,
                     resolution,
                     to_little(0x58595a49),
                     xyzi_size,
                     0,
                     voxels_n };
    std::vector<u32> buffer;
    buffer.reserve(total);
    for (auto&& s : header) buffer.push_back(s);
    for (u8 x = 0; x < resolution; x++)
        for (u8 y = 0; y < resolution; y++)
            for (u8 z = 0; z < resolution; z++)
                if (grid[x * resolution * resolution + y * resolution + z]) {
                    u32 position = (x << 24) + (y << 16) + (z << 8);
                    buffer.push_back(position);
                }
    fwrite(buffer.data(), sizeof(u32), buffer.size(), out);
    fclose(out);
    return 0;
}
