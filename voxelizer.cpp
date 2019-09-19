#include "voxelizer.h"
#include "math.h"
#include "types.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <tuple>
#include <utility>

using namespace IVoxelizer;
using Voxelizer::VoxelMeta;

void
flood_fill_rec(
    std::vector<Voxel_>& grid, const array<i32, 3>& grid_size, u32& voxels_n, i32 x, i32 y, i32 z)
{
    if (x < 0 || y < 0 || z < 0 || x >= grid_size.at(0) || y >= grid_size.at(1) || z >= grid_size.at(2))
        return;
    if (grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z).valid) return;

    grid.at(x * grid_size[1] * grid_size[2] + y * grid_size[2] + z).valid = true;
    voxels_n++;

    flood_fill_rec(grid, grid_size, voxels_n, x, y, z + 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y, z - 1);
    flood_fill_rec(grid, grid_size, voxels_n, x, y + 1, z);
    flood_fill_rec(grid, grid_size, voxels_n, x, y - 1, z);
    flood_fill_rec(grid, grid_size, voxels_n, x + 1, y, z);
    flood_fill_rec(grid, grid_size, voxels_n, x - 1, y, z);
}

void
flood_fill_rast(Voxel_* grid, const array<i32, 3>& grid_size, i32* voxels_n)
{
    struct {
        Voxel_::Type type;
        i32          z;
    } last;

    for (i32 x = 0; x < grid_size[0]; x++) {
        for (i32 y = 0; y < grid_size[1]; y++) {
            last = { Voxel_::CLOSING, std::numeric_limits<i32>::max() };
            for (i32 z = 0; z < grid_size[2]; z++) {
                i32     at    = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                Voxel_* voxel = &((Voxel_*)grid)[at];

                if (voxel->valid) { // NONE || BOTH || CLOSING
                    last = { voxel->max_type, z };
                } else if (last.type == Voxel_::OPENING && z > last.z) {
                    voxel->valid = true;
                    (*voxels_n)++;
                }
            }

            if (last.type == Voxel_::OPENING) printf("bad point: %d %d %d\n", x, y, last.z);
        }
    }
}

i32
flood_fill_rast2(u8 grid[], VoxelMeta data[], const array<i32, 3>& grid_size)
{
    i32 voxel_count = 0;

    struct {
        VoxelMeta::Type type;
        i32             y;
    } last;

    for (i32 z = 0; z < grid_size[0]; z++) {
        for (i32 x = 0; x < grid_size[1]; x++) {
            last = { VoxelMeta::CLOSING, std::numeric_limits<i32>::max() };
            for (i32 y = 0; y < grid_size[2]; y++) {
                i32             voxel_num = z * grid_size[1] * grid_size[2] + x * grid_size[2] + y;
                VoxelMeta::Type type      = data[voxel_num].type;
                if (get_voxel(grid, voxel_num)) {
                    switch (type) {
                    case VoxelMeta::OPENING: {
                        last.type = VoxelMeta::OPENING;
                        last.y    = y;
                        voxel_count++;
                    }; break;
                    case VoxelMeta::CROUDED: {
                        // voxel_count isn't incremented
                        set_voxel(grid, voxel_num, false);
                    }; break;
                    case VoxelMeta::CLOSING: {
                        last.type = VoxelMeta::CLOSING;
                        last.y    = y;
                        voxel_count++;
                    }; break;
                    case VoxelMeta::BOTH: {
                        voxel_count++;
                        last.y = y;
                    } break;
                    }
                } else if (last.type == VoxelMeta::OPENING && y > last.y) {
                    set_voxel(grid, voxel_num, true);
                    voxel_count++;
                }
            }

            // if (last.type == Voxel2::OPENING) printf("bad point: %d %d %d\n", x, y, last.z);
        }
    }

    return voxel_count;
}

void
grid_cut(std::vector<Voxel_>& grid,
         const array<i32, 3>& grid_size,
         i32&                 voxels_n,
         const array<i32, 3>& threshold)
{
    for (i32 x = 0; x < grid_size[0]; x++) {
        for (i32 y = 0; y < grid_size[1]; y++) {
            for (i32 z = 0; z < grid_size[2]; z++) {
                i32 at = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                if (x >= threshold[0] && y >= threshold[1] && z >= threshold[2] && grid.at(at).valid) {
                    grid.at(at).valid = false;
                    voxels_n--;
                }
            }
        }
    }
}

static inline bool
write_voxel(u8                   grid[],
            VoxelMeta            data[],
            const array<i32, 3>& grid_size,
            const vec3&          p,
            VoxelMeta::Type      type,
            unsigned int         tri_idx,
            bool                 flood_fill)
{
    bool   new_voxel = false;
    size_t voxel_num = std::floor(p[0]) * grid_size[1] * grid_size[2] + std::floor(p[1]) * grid_size[2] +
                       std::floor(p[2]);

    if (!get_voxel(grid, voxel_num)) {
        new_voxel = true;
        set_voxel(grid, voxel_num);
        if (data || flood_fill) {
            data[voxel_num].ids.push_back(tri_idx);
            data[voxel_num].type = type;
        }
    } else if (data || flood_fill) {
        data[voxel_num].ids.push_back(tri_idx);
        VoxelMeta::Type ctype = data[voxel_num].type;
        // if (ctype != type && type != VoxelMeta::BOTH && ctype != VoxelMeta::BOTH)
        // data[voxel_num].type = VoxelMeta::CROUDED;
        if ((ctype == VoxelMeta::Type::OPENING && type == VoxelMeta::Type::CLOSING) ||
            (ctype == VoxelMeta::Type::CLOSING && type == VoxelMeta::Type::OPENING))
            data[voxel_num].type = VoxelMeta::CROUDED;
    }

    return new_voxel;
}

int
Voxelizer::voxelize(unsigned char grid[],
                    int           grid_size_x,
                    int           grid_size_y,
                    int           grid_size_z,
                    float (*meshes[])[3][3],
                    size_t    meshes_size[],
                    size_t    meshes_count,
                    bool      flip_normals,
                    float     triangles_min[3],
                    float     triangles_max[3],
                    bool      flood_fill,
                    VoxelMeta data[])
{
    assert(!flood_fill || data);

    if (!triangles_min || !triangles_max) {
        f64 triangles_min_tmp[3] = { std::numeric_limits<f64>::max(), std::numeric_limits<f64>::max(),
                                     std::numeric_limits<f64>::max() };
        f64 triangles_max_tmp[3] = { -std::numeric_limits<f64>::max(), -std::numeric_limits<f64>::max(),
                                     -std::numeric_limits<f64>::max() };
        for (unsigned int m = 0; m < meshes_count; m++)
            for (unsigned int i = 0; i < meshes_size[m]; i++) {
                for (i32 k = 0; k < 3; k++) {
                    for (i32 j = 0; j < 3; j++) {
                        triangles_max_tmp[j] = std::max(triangles_max_tmp[j], meshes[m][i][k][j]);
                        triangles_min_tmp[j] = std::min(triangles_min_tmp[j], meshes[m][i][k][j]);
                    }
                }
            }
        triangles_max = triangles_max_tmp;
        triangles_min = triangles_min_tmp;
    }

    array<i32, 3> grid_size { grid_size_z, grid_size_x, grid_size_y };
    // swizzle(triangles_min, 3, 2);
    // swizzle(triangles_max, 3, 2);
    f64 triangles_min_min = std::min({ triangles_min[0], triangles_min[1], triangles_min[2] });
    f64 factor =
        std::max(triangles_max[0] - triangles_min[0],
                 std::max(triangles_max[1] - triangles_min[1], triangles_max[2] - triangles_min[2]));

    // normalize triangles
    for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
        for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
            Triangle& tri = *(Triangle*)(&meshes[mesh_it][tri_it]);

            if (flip_normals) std::swap(tri[1], tri[2]);
            for (auto& v : tri) {
                // v = swizzle(v, 2);
                // saves the headache of finding the grid voxel for each point
                for (i32 i = 0; i < 3; i++) {
                    // normalize to[0.0f, grid_size - 1.0f]
                    v[i] = (grid_size[i] - 1.0f) * (v[i] - triangles_min[i]) / factor;
                }
            }
        }
    }

    i32 voxel_count = 0;
    if (true) {
        for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
            for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
                Triangle& tri = *(Triangle*)(&meshes[mesh_it][tri_it]);
                // printf("tri[0]=%s tri[1]=%s tri[2]=%s ", tri[0].to_string().c_str(),
                // tri[1].to_string().c_str(), tri[2].to_string().c_str());

                vec3 tri_normal = cross(tri[1] - tri[0], tri[2] - tri[0]);
                // printf("normal=%s\n", tri_normal.to_string().c_str());
                VoxelMeta::Type type;
                if (data || flood_fill) {
                    if (std::abs(tri_normal.z) < epsilon)
                        type = VoxelMeta::BOTH;
                    else if (tri_normal.z < 0.0f)
                        type = VoxelMeta::OPENING;
                    else if (tri_normal.z > 0.0f)
                        type = VoxelMeta::CLOSING;
                }

                const f64* dominant_axis =
                    std::max_element(tri_normal.begin(), tri_normal.end(),
                                     [](f64 lhs, f64 rhs) { return abs(lhs) < abs(rhs); });
                i8 xi = 0, yi = 1, zi = dominant_axis - &tri_normal.x;
                if (zi == xi) xi = 2;
                if (zi == yi) yi = 2;
                assert(xi != yi && yi != zi && xi != zi);
                                
                f64 min_x = tri[0].x, max_x = tri[0].x, min_y = tri[0].y, max_y = tri[0].y;
                for (u8 vi = 1; vi < 3; vi++) {
                    min_x = std::min(min_x, tri[vi][xi]);
                    max_x = std::max(max_x, tri[vi][xi]);
                    min_y = std::min(min_y, tri[vi][yi]);
                    max_y = std::max(max_y, tri[vi][yi]);
                }
                if (max_x - min_x > max_y - min_y) {
                    // printf("swapped: ");
                    std::swap(xi, yi);
                }

                // printf("xi=%d yi=%d zi=%d\n", xi, yi, zi);
                std::sort(tri.begin(), tri.end(),
                          [=](const vec3& lhs, const vec3& rhs) { return lhs[yi] < rhs[yi]; });

                struct Edge {
                    f64 x;
                    f64 dx;
                } edges[2];

                edges[0].dx = (tri[2][xi] - tri[0][xi]) / (tri[2][yi] - tri[0][yi]);
                edges[0].x  = tri[0][xi];

                bool horizontal_first_edge = (tri[0][yi] == tri[1][yi]);
                if (horizontal_first_edge) {
                    edges[1].dx = (tri[2][xi] - tri[1][xi]) / (tri[2][yi] - tri[1][yi]);
                    edges[1].x  = tri[1][xi];
                } else {
                    edges[1].dx = (tri[1][xi] - tri[0][xi]) / (tri[1][yi] - tri[0][yi]);
                    edges[1].x  = tri[0][xi];
                }
                // printf("edges[0].dx=%f edges[1].dx=%f\n", edges[0].dx, edges[1].dx);
                const f64 y_presition = 1;
                for (f64 y = tri[0][yi]; y <= tri[2][yi]; y += 1.0f / y_presition) {
                    if (y >= tri[1][yi] && y - 1.0f < tri[1][yi]) {
                        edges[1].dx = (tri[2][xi] - tri[1][xi]) / (tri[2][yi] - tri[1][yi]);
                        edges[1].x  = tri[1][xi] + (y - tri[1][yi]) * edges[1].dx;
                        // printf("next edges[1].dx=%f\n", edges[1].dx);
                    }

                    // printf("y=%f\n", y);

                    for (f64 x = std::floor(std::min(edges[0].x, edges[1].x));
                         x <= std::max(edges[0].x, edges[1].x);
                         // TODO: change x += 1.0f to dx
                         x += 1.0f) {
                        // printf("\tx=%f\n", x);
                        vec3 p;
                        p[xi]       = x;
                        p[yi]       = y;
                        const f64 a = tri_normal[xi], b = tri_normal[yi], c = tri_normal[zi];
                        const f64 d = -dot(tri_normal, tri[0]);
                        p[zi]       = (-1.0f / c) * (a * x + b * y + d);
                        if (p[zi] >= 0.0) {
                            bool new_voxel =
                                write_voxel(grid, data, grid_size, p, type, tri_it, flood_fill);
                            if (new_voxel) voxel_count++;
                        }
                        // printf("\t\tp=%s\n", p.to_string().c_str());
                        // for (int i = 0; i < 3; i++) {
                        // printf("\t\tfor i=%d: p[%d]=%f (not floored: %f)\n", i, i, p[i],
                        // (-1.0f / tri_normal[zi]) * (tri_normal[xi] * x + tri_normal[yi] * y +
                        // -dot(tri_normal, tri[0])));
                        // fflush(stdout);
                        // }
                    }

                    for (auto& e : edges) e.x += e.dx / y_presition;
                }
            }
        }
        /*} else if (true) {
        for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
            for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
                Triangle& tri = *(Triangle*)(&meshes[mesh_it][tri_it]);

                vec3 tri_normal = cross(tri[1] - tri[0], tri[2] - tri[0]);
                VoxelMeta type;
                if (std::abs(tri_normal.z) < epsilon)
                    type = VoxelMeta::BOTH;
                else if (tri_normal.z < 0.0f)
                    type = VoxelMeta::OPENING;
                else if (tri_normal.z > 0.0f)
                    type = VoxelMeta::CLOSING;

                const f64* dominant_axis =
                    std::max_element(tri_normal.begin(), tri_normal.end(),
                                     [](f64 lhs, f64 rhs) { return abs(lhs) < abs(rhs); });
                i32 dominant_axis_idx = dominant_axis - &tri_normal.x;

                for (auto& v : tri) v = swizzle(v, (dominant_axis_idx + 1) % 3);
                tri_normal = swizzle(tri_normal, (dominant_axis_idx + 1) % 3);

                std::sort(tri.begin(), tri.end(),
                          [](const vec3& lhs, const vec3& rhs) { return lhs.y < rhs.y; });

                // assume scanline: edge[0] -> edge[1]
                // assert(tri[0].y < tri[2].y);
                // assert(tri[0].x != tri[1].x || tri[1].x != tri[2].x || tri[0].x != tri[2].x);
                // const auto signed_dist = signed_edge_function(tri[0], tri[2], true, tri[1]);
                // if (signed_dist == 0.0f) continue; // FIXME? add a warning
                // const f64 dx = signed_dist > 0.0f ? -1.0f : 1.0f;

                struct Edge {
                    f64 x;
                    f64 dx;
                } edges[2];

                constexpr f64 y_presition = 2.0f;
                edges[0].dx = (tri[2].x - tri[0].x) / (tri[2].y - tri[0].y);
                edges[0].x = tri[0].x;

                // traverse only
                bool horizontal_first_edge = (tri[0].y == tri[1].y);
                if (horizontal_first_edge) {
                    edges[1].dx = (tri[2].x - tri[1].x) / (tri[2].y - tri[1].y);
                    edges[1].x = tri[1].x;
                } else {
                    edges[1].dx = (tri[1].x - tri[0].x) / (tri[1].y - tri[0].y);
                    edges[1].x = tri[0].x;
                }

                for (f64 y = tri[0].y; y <= tri[2].y; y += 1.0f / y_presition) {
                    if (!horizontal_first_edge && y >= tri[1].y && y - 1.0f < tri[1].y) {
                        edges[1].dx = (tri[2].x - tri[1].x) / (tri[2].y - tri[1].y);
                        edges[1].x = tri[1].x + (y - tri[1].y) * edges[1].dx;
                    }

                    f64 grid_x = swizzle(grid_size, (dominant_axis_idx + 1) % 3)[0];
                    for (f64 x = progressive_floor(std::min(edges[0].x, edges[1].x));
                         x <= progressive_ceil(std::max(edges[0].x, edges[1].x), grid_x - 1);
                         x += 1.0f) {
                        // TODO: change x += 1.0f to dx
                        const f64 a = tri_normal.x, b = tri_normal.y, c = tri_normal.z;
                        const f64 d = -dot(tri_normal, tri[0]);
                        f64 z = std::floor((-1.0f / c) * (a * x + b * y + d));
                        if (z < 0.0) continue;

                        vec3 point;
                        point = swizzle(vec3(x, y, z), 2 - dominant_axis_idx);
                        size_t voxel_num = std::floor(point.x) * grid_size[1] * grid_size[2] +
                                           std::floor(point.y) * grid_size[2] + std::floor(point.z);

                        if (!flood_fill) {
                            if (!get_voxel(grid, voxel_num)) {
                                voxel_count++;
                                set_voxel(grid, voxel_num);
                            }
                        } else {
                            Voxel2* voxel = &((Voxel2*)grid)[voxel_num];
                            if (!get_voxel<Voxel2>((Voxel2*)grid, voxel_num)) {
                                set_voxel<Voxel2>((Voxel2*)grid, voxel_num);
                                voxel->type = type;
                                voxel_count++;
                            } else if (voxel->type != type && voxel->type != VoxelMeta::CROUDED &&
                                       (type == VoxelMeta::BOTH || voxel->type == VoxelMeta::BOTH)) {
                                voxel->type = VoxelMeta::CROUDED;
                            }

                            if (data) types[voxel_num] = voxel->type;
                        }
                    }

                    for (auto& e : edges) e.x += e.dx / y_presition;
                }
            }
        }*/
    } else {
        assert(flood_fill || data);
        Voxel_* grid_ = (Voxel_*)calloc(grid_size[0] * grid_size[1] * grid_size[2], sizeof(Voxel_));

        for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
            for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
                Triangle& tri = *(Triangle*)(&meshes[mesh_it][tri_it]);
                assert(std::all_of(tri.begin(), tri.end(), [&](const vec3& v) {
                    bool res = true;
                    for (int i = 0; i < 3; i++) res = res && v[i] >= 0.0 && v[i] < grid_size[i];
                    return res;
                }));

                vec3 tmin, tmax;
                triangle_aabb(tri, &tmin, &tmax);

                array<i32, 3> aligned_min, aligned_max;
                for (int i = 0; i < 3; i++) {
                    aligned_min[i] = static_cast<i32>(progressive_floor(tmin[i]));
                    aligned_max[i] = static_cast<i32>(std::floor(tmax[i]));
                }

                vec3 tri_normal = cross(tri[1] - tri[0], tri[2] - tri[0]);
                vec3 edges[3];
                for (int i = 0; i < 3; i++) edges[i] = tri[(i + 1) % 3] - tri[i];

                Voxel_::Type type;
                if (std::abs(tri_normal.z) < epsilon)
                    type = Voxel_::BOTH;
                else if (tri_normal.z < 0.0f)
                    type = Voxel_::OPENING;
                else if (tri_normal.z > 0.0f)
                    type = Voxel_::CLOSING;

                for (i32 z = aligned_min[0]; z <= aligned_max[0]; z++) {
                    for (i32 x = aligned_min[1]; x <= aligned_max[1]; x++) {
                        for (i32 y = aligned_min[2]; y <= aligned_max[2]; y++) {
                            auto           at = z * grid_size[1] * grid_size[2] + x * grid_size[2] + y;
                            vec3           min_voxel = vec3(z, x, y);
                            array<vec3, 2> aabb      = { min_voxel, min_voxel + 1.0f };

                            bool is_coll = triangle_aabb_collision_mt(tri, tri_normal, edges, aabb);
                            if (is_coll) {
                                Voxel_* voxel = (Voxel_*)&grid_[at];
                                if (!voxel->valid) {
                                    voxel_count++;
                                    voxel->valid = true;
                                }

                                auto colls = find_triangle_aabb_collision(tri, edges, aabb);
                                // if (colls.empty()) continue; // the triangle is inside
                                // the aabb
                                if (colls.empty()) colls.insert(colls.end(), tri.cbegin(), tri.cend());

                                auto max_coll = *std::max_element(colls.cbegin(), colls.cend(),
                                                                  [](const vec3& l, const vec3& r) {
                                                                      return l.z < r.z;
                                                                  });

                                // update only max offset collision
                                if (max_coll.z + epsilon < voxel->max_coll_off) continue;

                                // new voxel or prev collision is less than current max
                                // collision is updated
                                if (voxel->max_type == Voxel_::NONE ||
                                    max_coll.z > voxel->max_coll_off) {
                                    voxel->max_type     = type;
                                    voxel->max_coll_off = max_coll.z;
                                }
                                // collision points are equal, favor? closing triangles
                                // FIXME: add left-of / right-of collision type to decide
                                // which collision to favor
                                else if (type == Voxel_::CLOSING) {
                                    voxel->max_type     = type;
                                    voxel->max_coll_off = max_coll.z;
                                }

                                /*
                                   else if (max_coll.z == voxel->max_coll_off &&
                                   max_coll.z == aabb[1].z) { if (type == Voxel_::CLOSING)
                                   { voxel->max_type = type; voxel->max_coll_off =
                                   max_coll.z; }
                                   }
                                */
                            }
                        }
                    }
                }
            }
        }

        assert(data);
        for (int i = 0; i < grid_size[0] * grid_size[1] * grid_size[2]; i++) {
            if (grid_[i].valid) {
                set_voxel(grid, i, true);
                switch (grid_[i].max_type) {
                case Voxel_::CLOSING: data[i].type = VoxelMeta::CLOSING; break;
                case Voxel_::OPENING: data[i].type = VoxelMeta::OPENING; break;
                case Voxel_::BOTH: data[i].type = VoxelMeta::BOTH; break;
                default: break;
                }
            }
        }
        free(grid_);
    }

    if (flood_fill) voxel_count = flood_fill_rast2(grid, data, grid_size);

    return voxel_count;
}
