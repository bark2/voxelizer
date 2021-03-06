#include "voxelizer.h"
#include "vox_math.h"
#include "vox_types.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <random>
#include <tuple>
#include <utility>

#define vprintf(verbose, ...)                                                                           \
    if (verbose) printf(__VA_ARGS__);

using namespace IVoxelizer;
using Voxelizer::VoxelType;

static inline bool
is_closed_seed_scene(double (*meshes[])[3][3], size_t meshes_size[], size_t meshes_count, vec3 seed)
{
    array<vec3, 2> ray;
    vec3           tmp, p;
    unsigned int   coll_mesh_it, coll_tri_it;
    ray[0] = seed;

    for (f64 alpha = 0.0; alpha <= 180.0; alpha += 10.0) {
        // printf("alpha: %f\n", alpha);
        for (f64 beta = 0.0; beta < 360.0; beta += 1.0) {
            ray[1].x = seed.x + sin(alpha) * cos(beta);
            ray[1].y = seed.y + sin(alpha) * sin(beta);
            ray[1].z = seed.z + cos(alpha);

            bool found = false;
            for (unsigned int mesh_it = 0; mesh_it < meshes_count; mesh_it++)
                for (unsigned int tri_it = 0; tri_it < meshes_size[mesh_it]; tri_it++) {
                    Triangle& tri  = *(Triangle*)(&meshes[mesh_it][tri_it]);
                    auto      coll = ray_triangle_intersection_mt(ray, tri);
                    if (!coll.first) continue;
                    tmp = lerp(ray, coll.second);
                    if (!found || //
                        (found && (tmp - seed).squared_length() < (p - seed).squared_length())) {
                        found        = true;
                        p            = tmp;
                        coll_mesh_it = mesh_it;
                        coll_tri_it  = tri_it;
                    }
                }
            if (!found) return false;

            Triangle& tri = *(Triangle*)(&meshes[coll_mesh_it][coll_tri_it]);
            vec3      n   = cross(tri[1] - tri[0], tri[2] - tri[0]);
            if (dot(tri[0] - seed, n) <= 0.0) return false;
        }
    }

    return true;
}

static inline bool
is_closed_seed(double (*mesh)[3][3], size_t mesh_size, vec3 seed)
{
    array<vec3, 2> ray;
    vec3           tmp, p;
    unsigned int   coll_tri_it;
    ray[0] = seed;

    for (f64 alpha = 0.0; alpha <= 180.0; alpha += 10.0) {
        // printf("alpha: %f\n", alpha);
        for (f64 beta = 0.0; beta < 360.0; beta += 1.0) {
            ray[1].x = seed.x + sin(alpha) * cos(beta);
            ray[1].y = seed.y + sin(alpha) * sin(beta);
            ray[1].z = seed.z + cos(alpha);

            bool found = false;
            for (unsigned int tri_it = 0; tri_it < mesh_size; tri_it++) {
                Triangle& tri  = *(Triangle*)(&mesh[tri_it]);
                auto      coll = ray_triangle_intersection_mt(ray, tri);
                if (!coll.first) continue;
                tmp = lerp(ray, coll.second);
                if (!found || //
                    (found && (tmp - seed).squared_length() < (p - seed).squared_length())) {
                    found       = true;
                    p           = tmp;
                    coll_tri_it = tri_it;
                }
            }
            if (!found) return false;

            Triangle& ctri = *(Triangle*)(&mesh[coll_tri_it]);
            vec3      n    = cross(ctri[1] - ctri[0], ctri[2] - ctri[0]);
            if (dot(ctri[0] - seed, n) <= 0.0) return false;
        }
    }

    return true;
}

static inline void
flood_fill_rec_imp(u8 grid[], const array<i32, 3>& grid_size, unsigned int* voxel_count, i32 x, i32 y,
                   i32 z)
{
    if (x < 0 || y < 0 || z < 0 || x >= grid_size[0] || y >= grid_size[1] || z >= grid_size[2]) return;
    if (get_voxel(grid, x * grid_size[1] * grid_size[2] + y * grid_size[2] + z)) return;

    set_voxel(grid, x * grid_size[1] * grid_size[2] + y * grid_size[2] + z, true);
    (*voxel_count)++;

    flood_fill_rec_imp(grid, grid_size, voxel_count, x, y, z + 1);
    flood_fill_rec_imp(grid, grid_size, voxel_count, x, y, z - 1);
    flood_fill_rec_imp(grid, grid_size, voxel_count, x, y + 1, z);
    flood_fill_rec_imp(grid, grid_size, voxel_count, x, y - 1, z);
    flood_fill_rec_imp(grid, grid_size, voxel_count, x + 1, y, z);
    flood_fill_rec_imp(grid, grid_size, voxel_count, x - 1, y, z);
}

static inline size_t
calc_voxel_num(const array<i32, 3>& grid_size, const std::array<i32, 3>& v)
{
    return v[0] * grid_size[1] * grid_size[2] + v[1] * grid_size[2] + v[2];
}

static inline bool
legal_and_unset(u8 grid[], const array<i32, 3>& grid_size, array<i32, 3> v)
{
    bool legal = !(v[0] < 0 || v[1] < 0 || v[2] < 0 || v[0] >= grid_size[0] || v[1] >= grid_size[1] ||
                   v[2] >= grid_size[2]);
    if (!legal) return false;
    return !get_voxel(grid, grid_size, v);
}

static inline void
flood_fill_rec_imp_bfs(u8 grid[], u8 flood_fill_mem[], const array<i32, 3>& grid_size,
                       unsigned int* voxel_count, i32 x, i32 y, i32 z)
{
    unsigned int size = static_cast<unsigned int>(grid_size[0]) *
                        static_cast<unsigned int>(grid_size[1]) *
                        static_cast<unsigned int>(grid_size[2]);
    Voxel_Queue q((array<i32, 3>*)flood_fill_mem, size);
    assert(!get_voxel(grid, grid_size, { x, y, z }));

    set_voxel(grid, grid_size, { x, y, z }, true);
    (*voxel_count)++;
    q.push({ x, y, z });

    while (!q.empty()) {
        auto v = q.front();
        q.pop();

        array<i32, 3> nb[] = { { v[0], v[1], v[2] - 1 }, { v[0], v[1], v[2] + 1 },
                               { v[0], v[1] - 1, v[2] }, { v[0], v[1] + 1, v[2] },
                               { v[0] - 1, v[1], v[2] }, { v[0] + 1, v[1], v[2] } };
        for (const auto& v : nb) {
            if (!legal_and_unset(grid, grid_size, { v[0], v[1], v[2] })) continue;

            set_voxel(grid, grid_size, { v[0], v[1], v[2] }, true);
            (*voxel_count)++;
            q.push({ v[0], v[1], v[2] });
        }
    }
}

static inline unsigned int
flood_fill_rec_scene(u8 grid[], u8 flood_fill_mem[], double (*meshes[])[3][3], size_t meshes_size[],
                     size_t meshes_count, const array<i32, 3>& grid_size, unsigned int shell_voxel_count)
{
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<f64> d[3];
    for (int i = 0; i < 3; i++) d[i] = std::uniform_real_distribution<f64>(0.0, grid_size[i]);

    array<i32, 3> iseed;
    vec3          seed;
    do {
        for (int i = 0; i < 3; i++) seed[i] = d[i](generator);
        iseed = { static_cast<i32>(seed[0]), static_cast<i32>(seed[1]), static_cast<i32>(seed[2]) };
    } while (get_voxel(grid, grid_size, iseed) ||
             !is_closed_seed_scene(meshes, meshes_size, meshes_count, seed));

    // printf("using seed: %s\n", seed.to_string().c_str());
    unsigned int voxel_count = shell_voxel_count;
    flood_fill_rec_imp_bfs(grid, flood_fill_mem, grid_size, &voxel_count, static_cast<i32>(seed.x),
                           static_cast<i32>(seed.y), static_cast<i32>(seed.z));
    return voxel_count;
}

static inline unsigned int
flood_fill_rec(u8 grid[], u8 flood_fill_mem[], double (*mesh)[3][3], size_t mesh_size,
               const array<i32, 3>& grid_size, unsigned int shell_voxel_count)
{
    std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<f64> d[3];
    for (int i = 0; i < 3; i++) d[i] = std::uniform_real_distribution<f64>(0.0, grid_size[i]);

    array<i32, 3> iseed;
    vec3          seed;
    do {
        for (int i = 0; i < 3; i++) seed[i] = d[i](generator);
        iseed = { static_cast<i32>(seed[0]), static_cast<i32>(seed[1]), static_cast<i32>(seed[2]) };
    } while (get_voxel(grid, grid_size, iseed) || !is_closed_seed(mesh, mesh_size, seed));

    unsigned int voxel_count = shell_voxel_count;
    flood_fill_rec_imp_bfs(grid, flood_fill_mem, grid_size, &voxel_count, static_cast<i32>(seed.x),
                           static_cast<i32>(seed.y), static_cast<i32>(seed.z));

    return voxel_count;
}

static u8
flood_fill_rast_collision_detection(u8 grid[], VoxelData data[], const array<i32, 3>& grid_size,
                                    unsigned int* voxel_count)
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

static u32
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
                i32 voxel_num = z * grid_size[1] * grid_size[2] + x * grid_size[2] + y;
                if (get_voxel(grid, voxel_num)) {
                    VoxelType type = data[voxel_num];
                    switch (type) {
                    case VoxelType::OPENING: {
                        last.type = VoxelType::OPENING;
                        last.y    = y;
                        voxel_count++;
                    }; break;
                    case VoxelType::CROUDED: {
                        // voxel_count isn't incremented, and last type y value isnt updated
                        // set_voxel(grid, voxel_num, false);
                        last.y = y;
                        voxel_count++;
                        last.type = VoxelType::CLOSING;
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
        }
    }

    return voxel_count;
}

static inline bool
write_voxel_imp(u8 grid[], Voxelizer::VoxelType data[], const array<i32, 3>& grid_size, const vec3& p,
                Voxelizer::VoxelType type, bool flood_fill, bool verbose)
{
    bool          is_new_voxel = false;
    array<i32, 3> v;
    v[0]             = static_cast<i32>(floor(p[0]));
    v[1]             = static_cast<i32>(floor(p[1]));
    v[2]             = static_cast<i32>(floor(p[2]));
    size_t voxel_num = calc_voxel_num(grid_size, v);

    if (!get_voxel(grid, voxel_num)) {
        is_new_voxel = true;
        set_voxel(grid, voxel_num);

        if (data || flood_fill) {
            data[voxel_num] = type;
            vprintf(verbose, "new voxel type: %d\n", type)
        }
    }
    else if (data || flood_fill) {
        Voxelizer::VoxelType ctype = data[voxel_num];
        if ((ctype == Voxelizer::VoxelType::OPENING && type == Voxelizer::VoxelType::CLOSING) ||
            (ctype == Voxelizer::VoxelType::CLOSING && type == Voxelizer::VoxelType::OPENING))
            data[voxel_num] = Voxelizer::VoxelType::CROUDED;
        else if (ctype == Voxelizer::BOTH)
            data[voxel_num] = type;
    }
    return is_new_voxel;
}

static inline void
write_voxel(u8 grid[], Voxelizer::VoxelType data[], const array<i32, 3>& grid_size, const vec3& p,
            Voxelizer::VoxelType type, bool flood_fill, unsigned int* voxel_count, bool verbose)
{
    for (int i = 0; i < 3; i++) {
        if (p[i] >= 1.0 && floor(p[i]) == p[i]) {
            vec3 prev = p;
            prev[i] -= 1.0;
            (*voxel_count) +=
                write_voxel_imp(grid, data, grid_size, prev, type, flood_fill, verbose) ? 1 : 0;
        }
    }
    (*voxel_count) += write_voxel_imp(grid, data, grid_size, p, type, flood_fill, verbose) ? 1 : 0;
}

char
Voxelizer::voxelize(unsigned char grid[], unsigned int* voxel_count, int grid_size_x, int grid_size_y,
                    int grid_size_z, double (*meshes[])[3][3], u8 flip_normals[], size_t meshes_size[],
                    size_t meshes_count, double triangles_min[3], double triangles_max[3], FillType fill,
                    unsigned char flood_fill_mem[], bool use_collision_detection, unsigned char data[],
                    bool verbose)
{
    if (fill == FILL_SCANLINE && !data) return Voxelizer::ERROR_NO_DATA_BUFFER;
    if (use_collision_detection && !data) return Voxelizer::ERROR_NO_DATA_BUFFER;
    if (fill == FILL_FLOOD && !flood_fill_mem) return Voxelizer::ERROR_NO_DATA_BUFFER;

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
            if (flip_normals[mesh_it] == 1) std::swap(tri[1], tri[2]);
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
                const Triangle& orig       = *(Triangle*)(&meshes[mesh_it][tri_it]);
                Triangle        tri        = orig;
                vec3            tri_normal = cross(tri[1] - tri[0], tri[2] - tri[0]);
                if (tri_normal.length() < epsilon) {
                    printf("Warning: degenerated triangle is skipped\n");
                    continue;
                }

                VoxelType type;
                if (fill != FILL_NONE || data) {
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

                    vprintf(verbose, "\n");
                    vprintf(verbose, "tri: %u ", tri_it);
                    for (auto& v : tri) vprintf(verbose, "%s ", v.to_string().c_str());
                    vprintf(verbose, "\n");
                    vprintf(verbose, "edge[0]: %s\t%s\n", tri[0].to_string().c_str(),
                            tri[2].to_string().c_str());
                    vprintf(verbose, "edge[1]: %s\t%s\n", tri[0].to_string().c_str(),
                            tri[1].to_string().c_str());
                    vprintf(verbose, "edge[2]: %s\t%s\n", tri[1].to_string().c_str(),
                            tri[2].to_string().c_str());

                    const f64 a    = tri_normal[xi];
                    const f64 b    = tri_normal[yi];
                    const f64 c    = tri_normal[zi];
                    const f64 d    = -dot(tri_normal, tri[0]);
                    const f64 dzdx = -a / c;
                    const f64 dzdy = -b / c;
                    const f64 w    = -d / c;
                    vprintf(verbose, "a: %f,b: %f,c: %f,d: %f,dzdx: %f,dzdy: %f,w: %f\n", a, b, c, d,
                            dzdx, dzdy, w);

                    const f64 dx[3] = {
                        (tri[2][xi] - tri[0][xi]) / (tri[2][yi] - tri[0][yi]), // edge[0]
                        (tri[1][xi] - tri[0][xi]) / (tri[1][yi] - tri[0][yi]), // edge[1]
                        (tri[2][xi] - tri[1][xi]) / (tri[2][yi] - tri[1][yi])  // edge[2]
                    };

                    struct Edge {
                        f64 x;
                        f64 dx;
                    } edges[2];

                    edges[0].dx = dx[0];
                    edges[0].x  = tri[0][xi];

                    edges[1].dx                = dx[1];
                    edges[1].x                 = tri[0][xi];
                    bool horizontal_first_edge = (tri[0][yi] == tri[1][yi]);
                    vprintf(verbose, "horizontal first edge: %s\n",
                            horizontal_first_edge ? "true" : "false");
                    if (horizontal_first_edge) {
                        edges[1].dx = dx[2];
                        edges[1].x  = tri[1][xi];
                    }

                    u8 le = (tri[0][xi] + edges[0].dx * (tri[1][yi] - tri[0][yi]) < tri[1][xi] ? 0 : 1);
                    u8 re = (le == 0 ? 1 : 0);

                    vec3 curr;
                    // curr[zi] = dzdx * edges[le].x + dzdy * tri[0][yi] + w;
                    // TODO: force fill half first line, only first voxel is problematic
                    curr[yi] = ceil(tri[0][yi]);
                    for (auto& e : edges) e.x += e.dx * (curr[yi] - tri[0][yi]);
                    curr[zi] = (edges[le].dx * dzdx + dzdy) * (curr[yi] - tri[0][yi]) + tri[0][zi];

                    while (curr[yi] <= tri[2][yi]) {
                        if (curr[yi] >= tri[1][yi] && curr[yi] - 1.0 < tri[1][yi]) {
                            edges[1].dx = dx[2];
                            edges[1].x  = tri[1][xi] + (curr[yi] - tri[1][yi]) * dx[2];
                            if ((std::abs(curr[yi] - tri[1][yi]) < epsilon)) edges[1].x = tri[1][xi];
                            curr[zi] = dzdx * edges[le].x + dzdy * curr[yi] + w;
                            vprintf(verbose, "next edge: dx: %F, x: %F\n", edges[1].dx, edges[1].x);
                        }
                        vprintf(verbose, "curr[yi]: %F\n", curr[yi]);
                        f64 start_z = curr[zi];
                        for (curr[xi] = edges[le].x; curr[xi] <= edges[re].x;
                             curr[xi] += 1.0, curr[zi] += dzdx) {
                            if (verbose) {
                                vprintf(verbose, "\tcurr[xi]: %F\t", curr[xi]);
                                f64 z = dzdx * curr[xi] + dzdy * curr[yi] + w;
                                printf("curr[zi]:   %F,       z: %F\n", curr[zi], z);
                                if (curr[zi] - z > epsilon)
                                    vprintf(verbose, "\tWarning: curr[zi] != z\n");
                            }
                            if (curr[zi] < 0.0) continue;

                            write_voxel(grid, (VoxelType*)data, grid_size, curr, type, fill != FILL_NONE,
                                        voxel_count, verbose);
                        }

                        curr[zi] -= dzdx * (curr[xi] - edges[re].x);
                        curr[xi] = edges[re].x;
                        assert(abs(curr[zi] - (dzdx * curr[xi] + dzdy * curr[yi] + w)) < epsilon);
                        if (curr[zi] >= 0.0) {
                            write_voxel(grid, (VoxelType*)data, grid_size, curr, type, fill != FILL_NONE,
                                        voxel_count, verbose);
                        }

                        curr[yi] += 1.0;
                        for (auto& e : edges) e.x += e.dx;
                        curr[zi] = start_z + edges[le].dx * dzdx + dzdy;
                    }
                }
            }
        }
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

                                if (fill == FILL_NONE && !data) continue;
                                auto colls = find_triangle_aabb_collision(tri, edges, aabb);
                                // if (colls.empty()) continue; // the triangle is inside the aabb
                                if (colls.empty()) colls.insert(colls.end(), tri.cbegin(), tri.cend());

                                vec3 max_coll //
                                    = *std::max_element(colls.cbegin(), colls.cend(),
                                                        [=](const vec3& l, const vec3& r) {
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
    }

    if (fill == FILL_FLOOD) {
        (*voxel_count) = flood_fill_rec_scene(grid, flood_fill_mem, meshes, meshes_size, meshes_count,
                                              grid_size, *voxel_count);
    }
    else if (fill == FILL_FLOOD_MESHES) {
        for (size_t i = 0; i < meshes_count; i++) {
            f64(*mesh)[3][3] = meshes[i];
            size_t mesh_size = meshes_size[i];
            (*voxel_count) +=
                flood_fill_rec(grid, flood_fill_mem, mesh, mesh_size, grid_size, *voxel_count);
        }
    }
    else if (fill == FILL_SCANLINE) {
        if (use_collision_detection)
            return flood_fill_rast_collision_detection(grid, (VoxelData*)data, grid_size, voxel_count);
        else
            (*voxel_count) = flood_fill_rast(grid, (VoxelType*)data, grid_size);
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

size_t
Voxelizer::size_of_voxel_type_flood_fill()
{
    return 3 * sizeof(i32);
}
