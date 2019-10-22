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
using Voxelizer::VoxelType;

u8
flood_fill_rast_collision_detection(u8                   grid[],
                                    VoxelData            data[],
                                    const array<i32, 3>& grid_size,
                                    unsigned int*        voxel_count)
{
    u8 result = Voxelizer::SUCCESS;

    *voxel_count = 0;
    struct {
        VoxelData::Type type;
        i32             z;
    } last;
    for (i32 x = 0; x < grid_size[0]; x++) {
        for (i32 y = 0; y < grid_size[1]; y++) {
            last = { VoxelData::CLOSING, std::numeric_limits<i32>::max() };
            for (i32 z = 0; z < grid_size[2]; z++) {
                i32        at    = x * grid_size[1] * grid_size[2] + y * grid_size[2] + z;
                VoxelData* voxel = &data[at];

                if (get_voxel(grid, at)) { // NONE || BOTH || CLOSING
                    last = { voxel->max_type, z };
                    (*voxel_count)++;
                }
                else if (last.type == VoxelData::OPENING && z > last.z) {
                    set_voxel(grid, at, true);
                    (*voxel_count)++;
                }
            }

            if (last.type == VoxelData::OPENING) {
                printf("Warning: voxel with no closing match, (%d %d %d)\n", x, y, last.z);
                result = Voxelizer::ERROR_VOXEL_WITHOUT_A_MATCH;
            }
        }
    }

    return result;
}

u32
flood_fill_rast(u8 grid[], VoxelType data[], const array<i32, 3>& grid_size)
{
    u32 voxel_count = 0;

    struct {
        VoxelType type;
        i32       y;
    } last;

    for (i32 z = 0; z < grid_size[0]; z++) {
        for (i32 x = 0; x < grid_size[1]; x++) {
            last = { VoxelType::CLOSING, std::numeric_limits<i32>::max() };
            for (i32 y = 0; y < grid_size[2]; y++) {
                i32       voxel_num = z * grid_size[1] * grid_size[2] + x * grid_size[2] + y;
                VoxelType type      = data[voxel_num];
                if (get_voxel(grid, voxel_num)) {
                    switch (type) {
                    case VoxelType::OPENING: {
                        last.type = VoxelType::OPENING;
                        last.y    = y;
                        voxel_count++;
                    }; break;
                    case VoxelType::CROUDED: {
                        // voxel_count isn't incremented
                        set_voxel(grid, voxel_num, false);
                    }; break;
                    case VoxelType::CLOSING: {
                        last.type = VoxelType::CLOSING;
                        last.y    = y;
                        voxel_count++;
                    }; break;
                    case VoxelType::BOTH: {
                        voxel_count++;
                        last.y = y;
                    } break;
                    case VoxelType::NONE: assert(0);
                    }
                }
                else if (last.type == VoxelType::OPENING && y > last.y) {
                    set_voxel(grid, voxel_num, true);
                    voxel_count++;
                }
            }

            // if (last.type == Voxel2::OPENING) printf("bad point: %d %d %d\n", x, y, last.z);
        }
    }

    return voxel_count;
}

extern unsigned int qtri;
static inline bool
write_voxel(u8                   grid[],
            Voxelizer::VoxelType data[],
            const array<i32, 3>& grid_size,
            const vec3&          p,
            Voxelizer::VoxelType type,
            bool                 flood_fill)
{
    bool   new_voxel = false;
    size_t voxel_num =
        floor(p[0]) * grid_size[1] * grid_size[2] + floor(p[1]) * grid_size[2] + floor(p[2]);

    if (!get_voxel(grid, voxel_num)) {
        new_voxel = true;
        set_voxel(grid, voxel_num);
        if (data || flood_fill) data[voxel_num] = type;
    }
    else if (data || flood_fill) {
        Voxelizer::VoxelType ctype = data[voxel_num];
        if ((ctype == Voxelizer::VoxelType::OPENING && type == Voxelizer::VoxelType::CLOSING) ||
            (ctype == Voxelizer::VoxelType::CLOSING && type == Voxelizer::VoxelType::OPENING))
            data[voxel_num] = Voxelizer::VoxelType::CROUDED;
    }
    return new_voxel;
}

char
Voxelizer::voxelize(unsigned char grid[],
                    unsigned int* voxel_count,
                    int           grid_size_x,
                    int           grid_size_y,
                    int           grid_size_z,
                    double (*meshes[])[3][3],
                    size_t        meshes_size[],
                    size_t        meshes_count,
                    bool          flip_normals,
                    double        triangles_min[3],
                    double        triangles_max[3],
                    bool          flood_fill,
                    bool          use_collision_detection,
                    unsigned char data[])
{
    if (flood_fill && !data) return Voxelizer::ERROR_NO_DATA_BUFFER;
    if (use_collision_detection && !data) return Voxelizer::ERROR_NO_DATA_BUFFER;

    if (!triangles_min || !triangles_max) {
        triangles_min = (f64*)malloc(sizeof(triangles_min[0]) * 3);
        triangles_max = (f64*)malloc(sizeof(triangles_max[0]) * 3);
        for (int i = 0; i < 3; i++) {
            triangles_min[i] = std::numeric_limits<f64>::max();
            triangles_max[i] = -std::numeric_limits<f64>::max();
        }

        for (unsigned int m = 0; m < meshes_count; m++)
            for (unsigned int i = 0; i < meshes_size[m]; i++)
                for (i32 k = 0; k < 3; k++)
                    for (i32 j = 0; j < 3; j++) {
                        triangles_max[j] = std::max(triangles_max[j], meshes[m][i][k][j]);
                        triangles_min[j] = std::min(triangles_min[j], meshes[m][i][k][j]);
                    }
    }

    array<i32, 3> grid_size         = { grid_size_z, grid_size_x, grid_size_y };
    f64           triangles_min_min = std::min({ triangles_min[0], triangles_min[1], triangles_min[2] });
    f64           triangles_max_max = std::max({ triangles_max[0], triangles_max[1], triangles_max[2] });

    // normalize triangles
    for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
        for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
            Triangle& tri = *(Triangle*)(&meshes[mesh_it][tri_it]);
            if (flip_normals) std::swap(tri[1], tri[2]);
            for (auto& v : tri) {
                // saves the headache of finding the grid voxel for each point
                v = { v.z, v.x, v.y };
                for (i32 i = 0; i < 3; i++) {
                    v[i] = ((grid_size[i] - 1.0) / (triangles_max_max - triangles_min_min)) *
                           (v[i] - triangles_min_min);
                }
            }
        }
    }

    *voxel_count = 0;
    if (!use_collision_detection) {
        for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
            for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
                Triangle& tri        = *(Triangle*)(&meshes[mesh_it][tri_it]);
                vec3      tri_normal = cross(tri[1] - tri[0], tri[2] - tri[0]);
                VoxelType type;
                if (flood_fill) {
                    if (std::abs(tri_normal.z) < epsilon)
                        type = VoxelType::BOTH;
                    else if (tri_normal.z < 0.0)
                        type = VoxelType::OPENING;
                    else if (tri_normal.z > 0.0)
                        type = VoxelType::CLOSING;
                }

                const f64* dominant_axis =
                    std::max_element(tri_normal.begin(), tri_normal.end(),
                                     [](f64 lhs, f64 rhs) { return abs(lhs) < abs(rhs); });
                i8 xi = 0, yi = 1, zi = dominant_axis - &tri_normal.x;
                if (zi == xi)
                    xi = 2;
                else if (zi == yi)
                    yi = 2;
                assert(xi != yi && yi != zi && xi != zi);

                for (int dir_iter = 0; dir_iter < 2; dir_iter++, std::swap(xi, yi)) {
                    std::sort(tri.begin(), tri.end(),
                              [=](const vec3& lhs, const vec3& rhs) { return lhs[yi] < rhs[yi]; });

                    // printf("\n");
                    // printf("tri: %u ", tri_it);
                    // for (auto& v : tri) printf("%s ", v.to_string().c_str());
                    // printf("\n");

                    const f64 a    = tri_normal[xi];
                    const f64 b    = tri_normal[yi];
                    const f64 c    = tri_normal[zi];
                    const f64 d    = -dot(tri_normal, tri[0]);
                    const f64 dzdx = -a / c;
                    const f64 dzdy = -b / c;
                    const f64 w    = -d / c;

                    const f64 dx[3] = {
                        (tri[2][xi] - tri[0][xi]) / (tri[2][yi] - tri[0][yi]), //
                        (tri[1][xi] - tri[0][xi]) / (tri[1][yi] - tri[0][yi]), //
                        (tri[2][xi] - tri[1][xi]) / (tri[2][yi] - tri[1][yi])  //
                    };

                    struct Edge {
                        f64 x;
                        f64 dx;
                    } edges[2];

                    edges[0].dx = dx[0];
                    edges[0].x  = tri[0][xi];

                    bool horizontal_first_edge = (tri[0][yi] == tri[1][yi]);
                    if (horizontal_first_edge) {
                        edges[1].dx = dx[2];
                        edges[1].x  = tri[1][xi];
                    }
                    else {
                        edges[1].dx = dx[1];
                        edges[1].x  = tri[0][xi];
                    }

                    u8 le = (tri[0][xi] + edges[0].dx * (tri[1][yi] - tri[0][yi]) < tri[1][xi] ? 0 : 1);
                    u8 re = (le == 0 ? 1 : 0);

                    vec3 curr;
                    curr[zi] = dzdx * edges[le].x + dzdy * tri[0][yi] + w;

                    for (curr[yi] = tri[0][yi]; curr[yi] <= tri[2][yi];) {
                        if (curr[yi] >= tri[1][yi] && curr[yi] - 1.0 < tri[1][yi]) {
                            edges[1].dx = dx[2];
                            edges[1].x  = tri[1][xi] + (curr[yi] - tri[1][yi]) * edges[1].dx;
                            curr[zi]    = dzdx * edges[le].x + dzdy * curr[yi] + w;
                            // printf("next edge: dx: %F, x: %F\n", edges[1].dx, edges[1].x);
                        }
                        // printf("curr[yi]: %F\n", curr[yi]);
                        f64 start_z = curr[zi];
                        for (curr[xi] = edges[le].x; curr[xi] <= edges[re].x;
                             curr[xi] += 1.0, curr[zi] += dzdx) {
                            // printf("\tcurr[xi]: %F\t", curr[xi]);

                            // f64 z = dzdx * curr[xi] + dzdy * curr[yi] + w;
                            // printf("curr[zi]:   %F,       z: %F\n", curr[zi], z);
                            // if (curr[zi] - z > epsilon) printf("\tWarning: curr[zi] != z\n");
                            // fflush(stdout);
                            if (curr[zi] < 0.0) continue;

                            bool new_voxel =
                                write_voxel(grid, (VoxelType*)data, grid_size, curr, type, flood_fill);
                            if (new_voxel) (*voxel_count)++;
                        }

                        curr[zi] -= dzdx * (curr[xi] - edges[re].x);
                        curr[xi] = edges[re].x;
                        assert(abs(curr[zi] - (dzdx * curr[xi] + dzdy * curr[yi] + w)) < epsilon);
                        if (curr[zi] >= 0.0) {
                            bool new_voxel =
                                write_voxel(grid, (VoxelType*)data, grid_size, curr, type, flood_fill);
                            if (new_voxel) (*voxel_count)++;
                        }

                        for (auto& e : edges) e.x += e.dx;
                        curr[yi] += 1.0;
                        curr[zi] = start_z + edges[le].dx * dzdx + dzdy;
                    }

                    curr = tri[1];
                    bool new_voxel =
                        write_voxel(grid, (VoxelType*)data, grid_size, curr, type, flood_fill);
                    if (new_voxel) (*voxel_count)++;
                    curr      = tri[2];
                    new_voxel = write_voxel(grid, (VoxelType*)data, grid_size, curr, type, flood_fill);
                    if (new_voxel) (*voxel_count)++;
                }
            }
        }

        if (flood_fill) (*voxel_count) = flood_fill_rast(grid, (VoxelType*)data, grid_size);
    }
    else {
        for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++) {
            for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
                Triangle& tri = *(Triangle*)(&meshes[mesh_it][tri_it]);
                assert(std::all_of(tri.begin(), tri.end(), [&](const vec3& v) {
                    bool res = true;
                    for (int i = 0; i < 3; i++) { res = res && v[i] >= 0.0 && v[i] < grid_size[i]; }
                    return res;
                }));

                vec3 tmin, tmax;
                triangle_aabb(tri, &tmin, &tmax);

                array<i32, 3> aligned_min, aligned_max;
                for (int i = 0; i < 3; i++) {
                    aligned_min[i] = static_cast<i32>(progressive_floor(tmin[i]));
                    aligned_max[i] = static_cast<i32>(floor(tmax[i]));
                }

                vec3 tri_normal = cross(tri[1] - tri[0], tri[2] - tri[0]);
                vec3 edges[3];
                for (int i = 0; i < 3; i++) edges[i] = tri[(i + 1) % 3] - tri[i];

                const i32       rast_coord = 2;
                VoxelData::Type type;
                if (std::abs(tri_normal[rast_coord]) < epsilon)
                    type = VoxelData::BOTH;
                else if (tri_normal[rast_coord] < 0.0)
                    type = VoxelData::OPENING;
                else if (tri_normal[rast_coord] > 0.0)
                    type = VoxelData::CLOSING;

                for (i32 z = aligned_min[0]; z <= aligned_max[0]; z++) {
                    for (i32 x = aligned_min[1]; x <= aligned_max[1]; x++) {
                        for (i32 y = aligned_min[2]; y <= aligned_max[2]; y++) {
                            auto           at = z * grid_size[1] * grid_size[2] + x * grid_size[2] + y;
                            vec3           min_voxel = vec3(z, x, y);
                            array<vec3, 2> aabb      = { min_voxel, min_voxel + 1.0 };

                            bool is_coll = triangle_aabb_collision_mt(tri, tri_normal, edges, aabb);
                            if (is_coll) {
                                if (!get_voxel(grid, at)) {
                                    set_voxel(grid, at, true);
                                    (*voxel_count)++;
                                }

                                if (!flood_fill && !data) continue;
                                auto colls = find_triangle_aabb_collision(tri, edges, aabb);
                                // if (colls.empty()) continue; // the triangle is inside the aabb
                                if (colls.empty()) colls.insert(colls.end(), tri.cbegin(), tri.cend());

                                vec3 max_coll //
                                    = *std::max_element(colls.cbegin(), colls.cend(),
                                                        [=](const vec3& l, const vec3& r) {
                                                            // return l[0] < r[0]; FIXME
                                                            return l[rast_coord] < r[rast_coord];
                                                        });

                                VoxelData* voxel = &((VoxelData*)data)[at];
                                // update only max offset collision
                                if (max_coll[rast_coord] + epsilon < voxel->max_coll_off) continue;

                                // new voxel or prev collision is less than current max
                                // collision is updated
                                if (voxel->max_type == VoxelData::NONE ||
                                    max_coll[rast_coord] > voxel->max_coll_off) {
                                    voxel->max_type     = type;
                                    voxel->max_coll_off = max_coll[rast_coord];
                                }
                                // collision points are equal, favor? closing triangles
                                // FIXME: add left-of / right-of collision type to decide
                                // which collision to favor
                                else if (type == VoxelData::CLOSING) {
                                    voxel->max_type     = type;
                                    voxel->max_coll_off = max_coll[rast_coord];
                                }
                                /*
                                   else if (max_coll[rast_coord] == voxel->max_coll_off &&
                                   max_coll[rast_coord] == aabb[1][rast_coord]) { if (type ==
                                   VoxelData::CLOSING) { voxel->max_type = type; voxel->max_coll_off =
                                   max_coll[rast_coord]; }
                                   }
                                */
                            }
                        }
                    }
                }
            }
        }

        if (flood_fill)
            return flood_fill_rast_collision_detection(grid, (VoxelData*)data, grid_size, voxel_count);
    }

    return SUCCESS;
}

size_t
Voxelizer::size_of_voxel_type_presice()
{
    return sizeof(VoxelData);
}

size_t
Voxelizer::size_of_voxel_type()
{
    return sizeof(VoxelType);
}
