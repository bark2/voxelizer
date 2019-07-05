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

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

static const u32 max_line = 128;

enum Command { POSITION = 0, TEXTURE, NORMAL, FACE, GROUP, OBJECT, SMOOTH, COMMENT, COMMANDS_COUNT };

static const std::array<const std::string, COMMANDS_COUNT> commands_map { "v ", "vt ", "vn ", "f ",
                                                                          "g ", "o ",  "s ",  "#" };

std::vector<Mesh>
load_obj_file(const std::string& filename)
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cout << "Error: load_obj_file\tfile: " << __FILE__ << "\tline:" << __LINE__ << '\n';
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
            for (u32 i = 0; i < 3; i++) v[i] = strtof(strtok_update(line, " ", i_line).data(), nullptr);
            positions.push_back(v);
            for (size_t i = 0; i < 3; i++) {
                aabb_max[i] = std::max(aabb_max[i], v[i]);
                aabb_min[i] = std::min(aabb_min[i], v[i]);
            }
        } break;
        case TEXTURE: {
            vec2 v;
            for (u32 i = 0; i < 2; i++) v[i] = strtof(strtok_update(line, " ", i_line).data(), nullptr);
            uvs.push_back(v);
        } break;
        case NORMAL: {
            vec3 v;
            for (u32 i = 0; i < 3; i++) v[i] = strtof(strtok_update(line, " ", i_line).data(), nullptr);
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
                const size_t second_delimeter = line.find_first_of('/', first_delimeter + 1);
                const size_t ending_space = line.find_first_of(' ', second_delimeter + 1);

                indexes[0] = strtoul(line.substr(static_cast<size_t>(i_line), first_delimeter).c_str(),
                                     nullptr, 10) -
                             1;
                indexes[1] =
                    strtoul(
                        line.substr(static_cast<size_t>(first_delimeter + 1), second_delimeter).c_str(),
                        nullptr, 10) -
                    1;
                indexes[2] =
                    strtoul(line.substr(static_cast<size_t>(second_delimeter + 1), ending_space).c_str(),
                            nullptr, 10) -
                    1;

                face[i] = { positions[indexes[0]], normals[indexes[2]],
                            (vertex_properties_n == 3) ? uvs[indexes[1]] : vec2(0.0f, 0.0f) };

                for (i_line = ending_space; i_line < line.length() && isspace(line.at(i_line)); i_line++)
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

            meshes.push_back(
                { std::move(vertices), std::move(indices), vertex_properties_n == 3 ? true : false });
            vertex_properties_n = 0;
        } break;
        default:;
        } // switch(c)
    }
    infile.close();

    if (!vertices.empty())
        meshes.push_back(
            { std::move(vertices), std::move(indices), vertex_properties_n == 3 ? true : false });

    vec3 aabb_range = aabb_max - aabb_min;
    f32 aabb_max_size = std::max({ aabb_range.x, aabb_range.y, aabb_range.z });
    for (auto& m : meshes) {
        for (auto& v : m.vertices) {
            assert(0);
            // fill
            assert(v.pos.x <= 1 && v.pos.y <= 1 && v.pos.z <= 1);
        }
    }

    return std::move(meshes);
}

Mesh
processMesh(aiMesh* mesh, const aiScene* scene)
{
    std::vector<Vertex> vertices;
    std::vector<u32> indices;

    for (u32 i = 0; i < mesh->mNumVertices; i++) {
        Vertex vertex;
        vertex.pos = { mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z };
        // vertex.normal = { mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z };
        vertices.push_back(vertex);
    }
    for (u32 i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        for (u32 j = 0; j < face.mNumIndices; j++) indices.push_back(face.mIndices[j]);
    }
    return { vertices, indices, false };
}

void
processNode(aiNode* node, const aiScene* scene, std::vector<Mesh>* meshes)
{
    for (u32 i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        meshes->emplace_back(processMesh(mesh, scene));
    }
    for (u32 i = 0; i < node->mNumChildren; i++) { processNode(node->mChildren[i], scene, meshes); }
}

std::vector<Mesh>
ai_load_obj_file(const std::string& filename)
{
    std::vector<Mesh> meshes;
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        printf("ERROR::ASSIMP::%s", importer.GetErrorString());
        return {};
    }
    processNode(scene->mRootNode, scene, &meshes);

    return meshes;
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
export_magicavoxel(const std::string& filename,
                   const std::vector<Voxel>& grid,
                   array<i32, 3> grid_size,
                   u32 voxels_n)
{
    FILE* out = fopen(filename.c_str(), "wb");
    if (!out) return 1;

    u32 meta_size = 3 * sizeof(u32);
    u32 size_size = 3 * sizeof(u32);
    u32 max_grid_size = std::max({ grid_size[0], grid_size[1], grid_size[2] });
    array<u32, 3> scaling = { max_grid_size / grid_size[0], max_grid_size / grid_size[1],
                              max_grid_size / grid_size[2] };
    voxels_n *= scaling[0] * scaling[1] * scaling[2];
    u32 xyzi_size = (voxels_n + 1) * sizeof(u32);
    u32 header_size = 56;
    u32 total = header_size + xyzi_size;
    u32 main_children_size = total - meta_size - 8;

    u32 header[] = { to_little(0x564f5820),
                     150,
                     to_little(0x4d41494e),
                     0,
                     main_children_size,
                     to_little(0x53495a45),
                     size_size,
                     0,
                     static_cast<u32>(grid_size[0]),
                     static_cast<u32>(grid_size[1]),
                     static_cast<u32>(grid_size[2]),
                     to_little(0x58595a49),
                     xyzi_size,
                     0,
                     voxels_n };
    std::vector<u32> buffer;
    buffer.reserve(total);
    for (auto&& s : header) buffer.emplace_back(s);

    for (u8 x = 0; x < grid_size[0]; x++)
        for (u8 y = 0; y < grid_size[1]; y++)
            for (u8 z = 0; z < grid_size[2]; z++) {
                u32 at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                if (grid.at(at).valid)
                    for (u8 i = 0; i < scaling[0]; i++)
                        for (u8 j = 0; j < scaling[1]; j++)
                            for (u8 k = 0; k < scaling[2]; k++) {
                                u8 color;
                                switch (grid.at(at).max_type) {
                                // case Voxel::OPENING: color = 121; break;
                                case Voxel::OPENING: color = 1; break;
                                // case Voxel::CLOSING: color = 28; break;
                                // case Voxel::NONE: color = 28; break;
                                case Voxel::CLOSING: color = 112; break;
                                case Voxel::NONE: color = 112; break; // gray
                                case Voxel::BOTH: color = 248; break;
                                default: {
                                    // printf("%d\n", grid.at(at).max_type);
                                    std::raise(SIGINT);
                                }
                                }
                                u32 position_color = ((x * scaling[0] + i) << 24) +
                                                     ((y * scaling[1] + j) << 16) +
                                                     ((z * scaling[2] + k) << 8) + color;
                                buffer.emplace_back(to_little(position_color));
                            }
            }

    fwrite(buffer.data(), sizeof(u32), buffer.size(), out);
    fclose(out);
    return 0;
}

int
export_raw(const std::vector<Voxel>& grid, array<i32, 3> grid_size)
{
    // printf("%d %d %d\n", grid_size[0], grid_size[1], grid_size[2]);
    for (i32 x = 0; x < grid_size[0]; x++)
        for (i32 y = 0; y < grid_size[1]; y++)
            for (i32 z = 0; z < grid_size[2]; z++) {
                u32 at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                u8 color;
                if (!grid.at(at).valid) {
                    putchar('0');
                } else {
                    putchar('1');
                    switch (grid.at(at).max_type) {
                    case Voxel::OPENING: color = 121; break;
                    case Voxel::CLOSING: color = 15; break;
                    case Voxel::NONE: {
                        color = 1;
                        // printf("NONE: %d %d %d\n", x, y, z);
                    } break; // gray
                    case Voxel::BOTH: color = 248; break;
                    default: {
                        assert(0);
                    }
                    }
                    putchar(color);
                }
            }
    return 0;
}
